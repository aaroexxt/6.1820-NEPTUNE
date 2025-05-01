import UIKit
import AVFoundation
import Accelerate
import Foundation
import CoreMotion

class NodeSimulatorViewController: UIViewController {
    
    let motionManager = CMMotionManager()
    
    var speakerAudioEngine = AVAudioEngine()
    var sourceNode: AVAudioSourceNode! // node to play/stop tones from
    var microphoneAudioEngine = AVAudioEngine()
    var selectedNodeID = "01"
    var fftSetup: FFTSetup!
    
    var audioWrapper: AudioWrapper!
    
    @IBOutlet weak var btn_select_id: UIButton!
    @IBAction func nodeIDSelection(_ sender: UIAction) {
        print("Selecting node ID: ", sender.title)
        if(sender.title == "Node 1") {
            self.selectedNodeID = "01"
        }
        else {
            self.selectedNodeID = "10"
        }
        self.btn_select_id.setTitle(sender.title, for: .normal)
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.sourceNode = AVAudioSourceNode { _, _, _, _ in return noErr }
        self.audioWrapper = AudioWrapper(sourceNode: self.sourceNode)
        
        requestMicrophoneAccess() // start recording
    }
    
    /*
     Get the current acceleration reading of the phone
     */
    func getAccelX(completion: @escaping (Double) -> Void) {
        guard motionManager.isAccelerometerAvailable else {
            print("Accelerometer is not available.")
            completion(-1.0)
            return
        }

        motionManager.accelerometerUpdateInterval = 0.1
        motionManager.startAccelerometerUpdates(to: .main) { data, error in
            defer {
                self.motionManager.stopAccelerometerUpdates()
            }

            if let accelData = data {
                print("Acceleration X: \(accelData.acceleration.x)")
                completion(accelData.acceleration.x)
            } else {
                print("Error getting acceleration data: \(String(describing: error))")
                completion(-1.0)
            }
        }
    }
    
    /*
     Runs on messages received from the microphone. Converts the list of bit values into its node id bits, message bits, hamming bits, and parity bit.
     Checks the message and sends a response accordingly.
     */
    func decodeMessage(msg: [Int], speakerAudioEngine: AVAudioEngine, sourceNode: inout AVAudioSourceNode) {
        if msg.count != 15 {
            return
        }
        
        // deconstruct the received message into its components
        let nodeID = Array(msg.prefix(2))
        let message = Array(msg[2...5] + msg[7...9] + [msg[11]])
        let hamming = Array([msg[6]] + [msg[10]] + msg[12...13])
        let parityBit = msg[14]
        
        print("Message Received! Node ID: ", nodeID, " Message: ", message, " Hamming: ", hamming, " Parity Bit: ", parityBit)
        
        // error-checking using hamming + parity bits
        let idAndMsgBits = nodeID + message
        let recievedHamming = getHammingBits(idAndMsgBits: idAndMsgBits)
        let allOtherBits = Array(msg[0...13])
        let recievedParity = getOverallParityBit(allOtherBits: allOtherBits)
        let isValid = (recievedHamming == hamming && recievedParity == parityBit)
        print("Is valid??? ", isValid)
        
        if(isValid) {
            let stringID = nodeID.map { String($0) }.joined()
            let stringMessage = message.map { String($0) }.joined()
            
            if(stringID != "00") { // ignore messages that are sent by the simulator
                
                // message asking if node (selected id) is connected
                if(stringMessage == "00000000") {
                    let responseMessage = constructMessage(id: self.selectedNodeID, messageBits: "01010101")
                    sendMessage(msgToSend: responseMessage, speakerAudioEngine: speakerAudioEngine, audioWrapper: audioWrapper)
                }
                
                else {
                    let allValues = Array(dataRequestMessageBits.values)
                    if(allValues.contains(stringMessage)) {
                        let valToSend = Int.random(in: 0..<100)
                        print("Sending data from simulator: ", valToSend)
                        let valAsDouble = Double(valToSend)
                        let scaled = Int(((valAsDouble + 2.0) / 4.0) * 255.0)
                        let binaryString = String(scaled, radix: 2).leftPadding(toLength: 8, withPad: "0")
                        let responseMessage = constructMessage(id: "00", messageBits: binaryString)
                        sendMessage(msgToSend: responseMessage, speakerAudioEngine: speakerAudioEngine, audioWrapper: self.audioWrapper)
                    }
                }
                
                // message asking for temperature reading
                /*
                else if(stringMessage == "00001101") {
                    print("Simulator was asked for acceleration data!!")
                    getAccelX { accelX in
                        let scaled = Int((accelX + 2.0) / 4.0 * 255.0)
                        let binaryString = String(scaled, radix: 2).leftPadding(toLength: 8, withPad: "0")
                        let responseMessage = constructMessage(id: "00", messageBits: binaryString)
                        sendMessage(msgToSend: responseMessage, speakerAudioEngine: speakerAudioEngine, audioWrapper: self.audioWrapper)
                    }
                }
                 */
                
                
            }
        }
        
    }

    /*
     Helper function needed for microphone use.
     */
    func requestMicrophoneAccess() {
        AVAudioSession.sharedInstance().requestRecordPermission { [weak self] granted in
            DispatchQueue.main.async {
                if granted {
                    self?.startAudioEngine() // start reading
                } else {
                    print("Microphone permission not granted.")
                }
            }
        }
    }

    /*
     Starts recording from microphone, identifying frequencies using an fft, finds which tone this coorepsonds to (start, end, high, low), decodes message once reaches 15 bits.
     */
    func startAudioEngine() {
            print("FFT Size: ", fftSize)
            let bufferSize = AVAudioFrameCount(fftSize)
            let log2n = vDSP_Length(log2(Float(fftSize)))

            // FFT setup (reuse across buffers)
            self.fftSetup = vDSP_create_fftsetup(log2n, FFTRadix(FFT_RADIX2))!

            // Prepare the window function once
            var window = [Float](repeating: 0, count: fftSize)
            vDSP_hann_window(&window, vDSP_Length(fftSize), Int32(vDSP_HANN_NORM))

            // Date formatter reused for debugging
            let formatter = DateFormatter()
            formatter.dateFormat = "HH:mm:ss"

            let inputNode = microphoneAudioEngine.inputNode
            let inputFormat = inputNode.outputFormat(forBus: 0)
            var readInput: [Int] = []
            var waitingForStart = true

            inputNode.removeTap(onBus: 0)
            inputNode.installTap(onBus: 0, bufferSize: bufferSize, format: inputFormat) { (buffer, time) in
                guard let channelData = buffer.floatChannelData?[0] else { return }

                // Windowed samples
                var samples = [Float](repeating: 0.0, count: fftSize)
                vDSP_vmul(channelData, 1, window, 1, &samples, 1, vDSP_Length(fftSize))

                // FFT setup
                var real = [Float](repeating: 0.0, count: fftSize/2)
                var imag = [Float](repeating: 0.0, count: fftSize/2)
                var complexBuffer = DSPSplitComplex(realp: &real, imagp: &imag)

                samples.withUnsafeBufferPointer { ptr in
                    ptr.baseAddress!.withMemoryRebound(to: DSPComplex.self, capacity: fftSize) { complexPtr in
                        vDSP_ctoz(complexPtr, 2, &complexBuffer, 1, vDSP_Length(fftSize / 2))
                    }
                }

                vDSP_fft_zrip(self.fftSetup, &complexBuffer, 1, log2n, FFTDirection(FFT_FORWARD))

                var magnitudes = [Float](repeating: 0.0, count: fftSize/2)
                vDSP_zvmags(&complexBuffer, 1, &magnitudes, 1, vDSP_Length(fftSize/2))

                let binCutoffFrequency: Float = 1000
                let sampleRate = Float(inputFormat.sampleRate)
                let binCutoffIndex = Int(binCutoffFrequency / (sampleRate / Float(fftSize)))
                let searchMagnitudes = magnitudes[binCutoffIndex...]
                let maxMag = searchMagnitudes.max() ?? 0.0

                if let maxIndex = searchMagnitudes.firstIndex(of: maxMag) {
                    let frequency = Float(maxIndex + binCutoffIndex) * sampleRate / Float(fftSize)
                    let currentTime = formatter.string(from: Date())
                    print("Frequency: \(frequency), Time: \(currentTime)")

                    // Frequency decoding
                    if abs(frequency - Float(endFrequency)) < lenianceValue {
                        self.decodeMessage(msg: readInput, speakerAudioEngine: self.speakerAudioEngine, sourceNode: &self.sourceNode)
                        readInput = []
                    }
                    if abs(frequency - Float(startFrequency)) < lenianceValue {
                        waitingForStart = false
                    }
                    if abs(frequency - Float(highFrequency)) < lenianceValue && !waitingForStart {
                        readInput.append(1)
                        waitingForStart = true
                        print(readInput)
                    }
                    if abs(frequency - Float(lowFrequency)) < lenianceValue && !waitingForStart {
                        readInput.append(0)
                        waitingForStart = true
                        print(readInput)
                    }
                }
            }

            do {
                let session = AVAudioSession.sharedInstance()
                try session.setCategory(.playAndRecord, mode: .default, options: [.defaultToSpeaker])
                try session.setActive(true)
                try session.setPreferredIOBufferDuration(0.001)
                print("Actual IO Buffer Duration: \(session.ioBufferDuration)")
                try session.overrideOutputAudioPort(.speaker)
                try microphoneAudioEngine.start()
            } catch {
                print("Audio session configuration error: \(error)")
            }
        }




    /*
     Stops the microphone when switching out of/closing this view
     */
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        microphoneAudioEngine.inputNode.removeTap(onBus: 0)
        microphoneAudioEngine.stop()
        vDSP_destroy_fftsetup(fftSetup) // Add this line if fftSetup is stored
    }

    
}

// extension for converting sensor readings to binary strings
extension String {
    func leftPadding(toLength: Int, withPad: String) -> String {
        let paddingCount = toLength - self.count
        if paddingCount > 0 {
            return String(repeating: withPad, count: paddingCount) + self
        } else {
            return String(self.suffix(toLength))
        }
    }
}
 
