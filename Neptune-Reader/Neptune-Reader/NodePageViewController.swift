//
//  NodePageViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/2/25.
//

import UIKit
import AVFoundation
import Accelerate

class NodePageViewController: UIViewController {
    
    @IBOutlet weak var nodeLabel: UILabel!
    @IBOutlet var dataTable: UITableView!
    

    @IBOutlet weak var btn_select_data: UIButton!
    @IBAction func dataToRefreshSelection(_ sender: UIAction) {
        print("Selected data to refresh option: ", sender.title)
        self.btn_select_data.setTitle(sender.title, for: .normal)
        self.selectedDataOption = sender.title
    }
    
    var speakerAudioEngine = AVAudioEngine()
    var sourceNode: AVAudioSourceNode! // node to play/stop tones from
    
    var microphoneAudioEngine = AVAudioEngine()
    
    var readInput: [Int] = []
    
    var audioWrapper: AudioWrapper!
    
    struct SensorReading {
        var sensorName: String
        var sensorValue: Double
    }
    
    var state: readerState = readerState.IDLE
    var selectedDataOption: String = "Temperature"

    // initializing a list of all the sensors on the node
    var sensorReadings: [SensorReading] = [
        SensorReading(sensorName: "Gyro X", sensorValue: 0.0),
        SensorReading(sensorName: "Gyro Y", sensorValue: 0.0),
        SensorReading(sensorName: "Gyro Z", sensorValue: 0.0),
        SensorReading(sensorName: "Acceleration X", sensorValue: 0.0),
        SensorReading(sensorName: "Acceleration Y", sensorValue: 0.0),
        SensorReading(sensorName: "Acceleration Z", sensorValue: 0.0),
        SensorReading(sensorName: "Magnetic Field X", sensorValue: 0.0),
        SensorReading(sensorName: "Magnetic Field Y", sensorValue: 0.0),
        SensorReading(sensorName: "Magnetic Field Z", sensorValue: 0.0),
        SensorReading(sensorName: "Orientation X", sensorValue: 0.0),
        SensorReading(sensorName: "Orientation Y", sensorValue: 0.0),
        SensorReading(sensorName: "Orientation Z", sensorValue: 0.0),
        SensorReading(sensorName: "Temperature", sensorValue: 0.0),
    ]
    
    var selectedNode: String? // the name of this node (passed in from viewController)

    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.sourceNode = AVAudioSourceNode { _, _, _, _ in return noErr }
        self.audioWrapper = AudioWrapper(sourceNode: self.sourceNode)

        // setup UI title and data table
        nodeLabel.text = selectedNode
        dataTable.delegate = self
        dataTable.dataSource = self
        
        // get rid of whitespace on table
        let v = UIView()
        v.backgroundColor = .clear
        dataTable.tableFooterView = v
        dataTable.backgroundColor = .clear
        dataTable.backgroundColor = .clear
        
        requestMicrophoneAccess(stopListeningImmediately: true)
    }
    
    
    @IBAction func refreshButtonTapped(_ sender: UIButton) {
        self.readInput = [] // reset the buffer
        
        // asking node 1 for temperature reaading
        let requestMessageBits = dataRequestMessageBits[selectedDataOption]
        let askMessage = constructMessage(id: "01", messageBits: requestMessageBits!)
        sendMessage(msgToSend: askMessage, speakerAudioEngine: self.speakerAudioEngine, audioWrapper: self.audioWrapper)
        
        self.state = readerState.LISTENING
        requestMicrophoneAccess(stopListeningImmediately: false) // start listening
    }
    
    /*
     Helper function to update correct entry in the UI table
     */
    func updateSensorReadingsWithValue(valUnscaled: Double) {
        for i in 0...(sensorReadings.count-1) {
            if(sensorReadings[i].sensorName == selectedDataOption) {
                let name = sensorReadings[i].sensorName
                print("updating sensor readings table for: ", name)
                let scale = dataScales[name]
                let val = scaleValue(val: valUnscaled, outMin: scale![0], outMax: scale![1])
                sensorReadings[i] = SensorReading(sensorName: name, sensorValue: val)
                DispatchQueue.main.async {
                    self.dataTable.reloadData()
                }
            }
        }
    }
    
    func scaleValue(val: Double, outMin: Double, outMax: Double) -> Double {
        return outMin + (val / 255) * (outMax - outMin)
    }
    
    func decodeMessage(msg: [Int], speakerAudioEngine: inout AVAudioEngine, sourceNode: inout AVAudioSourceNode) {
        if msg.count != 15 {
            return
        }
        let nodeID = Array(msg.prefix(2))
        let message = Array(msg[2...5] + msg[7...9] + [msg[11]])
        let hamming = Array([msg[6]] + [msg[10]] + msg[12...13])
        let parityBit = msg[14]
        print("Message Received! Node ID: ", nodeID, " Message: ", message, " Hamming: ", hamming, " Parity Bit: ", parityBit)
        
        let idAndMsgBits = nodeID + message
        let recievedHamming = getHammingBits(idAndMsgBits: idAndMsgBits)
        let allOtherBits = Array(msg[0...13])
        let recievedParity = getOverallParityBit(allOtherBits: allOtherBits)
        
        print("recieved hamming: ", recievedHamming, " recieved parity: ", recievedParity)
        let isValid = (recievedHamming == hamming && recievedParity == parityBit)
        print("Is valid??? ", isValid)
        
        if(isValid) {
            let stringID = nodeID.map { String($0) }.joined()
            if(stringID == "00") {
                let messageDataAsInteger = message.reduce(0) { ($0 << 1) | $1 }
                updateSensorReadingsWithValue(valUnscaled: Double(messageDataAsInteger))
            }
        }
    }
    
    /*
     Helper function needed for microphone use.
     */
    func requestMicrophoneAccess(stopListeningImmediately: Bool) {
        AVAudioSession.sharedInstance().requestRecordPermission { [weak self] granted in
            DispatchQueue.main.async {
                if granted {
                    self?.startAudioEngine(stopListeningImmediately: stopListeningImmediately) // start reading
                } else {
                    print("Microphone permission not granted.")
                }
            }
        }
    }
    
    /*
     Starts recording from microphone, identifying frequencies using an fft, finds which tone this coorepsonds to (start, end, high, low), decodes message once reaches 15 bits.
     */
    func startAudioEngine(stopListeningImmediately: Bool) {
        
        print("starting to listen...")
        // microphone setup
        let inputNode = microphoneAudioEngine.inputNode
        let inputFormat = inputNode.outputFormat(forBus: 0)
        var waitingForStart = true
        let bufferSize = AVAudioFrameCount(fftSize)
        
        // start recording...
        inputNode.removeTap(onBus: 0)
        inputNode.installTap(onBus: 0, bufferSize: bufferSize, format: inputFormat) { (buffer, time) in
            guard let channelData = buffer.floatChannelData?[0] else { return } // recorded data

            // fft stuff:
            var window = [Float](repeating: 0, count: fftSize)
            vDSP_hann_window(&window, vDSP_Length(fftSize), Int32(vDSP_HANN_NORM))

            var samples = [Float](repeating: 0.0, count: fftSize)
            vDSP_vmul(channelData, 1, window, 1, &samples, 1, vDSP_Length(fftSize))

            var real = [Float](repeating: 0.0, count: fftSize/2)
            var imag = [Float](repeating: 0.0, count: fftSize/2)

            var complexBuffer = DSPSplitComplex(realp: &real, imagp: &imag)
            samples.withUnsafeBufferPointer { ptr in
                ptr.baseAddress!.withMemoryRebound(to: DSPComplex.self, capacity: fftSize) { complexPtr in
                    vDSP_ctoz(complexPtr, 2, &complexBuffer, 1, vDSP_Length(fftSize / 2))
                }
            }

            let fftSetup = vDSP_create_fftsetup(vDSP_Length(log2(Float(fftSize))), FFTRadix(FFT_RADIX2))!
            vDSP_fft_zrip(fftSetup, &complexBuffer, 1, vDSP_Length(log2(Float(fftSize))), FFTDirection(FFT_FORWARD))

            var magnitudes = [Float](repeating: 0.0, count: fftSize/2)
            vDSP_zvmags(&complexBuffer, 1, &magnitudes, 1, vDSP_Length(fftSize/2))
            
            let binCutoffFrequency: Float = 1000 // discard low bins, can adjust this freq later
            let binCutoffIndex = Int(binCutoffFrequency / (Float(sampleRate) / Float(fftSize)))
            let searchMagnitudes = magnitudes[binCutoffIndex...]
            let maxMag = searchMagnitudes.max() ?? 0.0
            
            if let maxIndex = searchMagnitudes.firstIndex(of: maxMag) {
                let sampleRate = Float(inputFormat.sampleRate)
                let frequency = Float(maxIndex) * sampleRate / Float(fftSize)
                
                let formatter = DateFormatter()
                formatter.dateFormat = "HH:mm:ss"  // or "yyyy-MM-dd HH:mm:ss" for full date and time
                let currentTime = formatter.string(from: Date())

                print("freq: \(frequency), time: \(currentTime)")
                
                // compare frequency to target values:
                if(abs(frequency - Float(endFrequency)) < lenianceValue) {
                    self.decodeMessage(msg: self.readInput, speakerAudioEngine: &self.speakerAudioEngine, sourceNode: &self.sourceNode)
                    self.readInput = []
                }
                if (abs(frequency - Float(startFrequency)) < lenianceValue) {
                    waitingForStart = false
                }
                
                if (abs(frequency - Float(highFrequency)) < lenianceValue && (!waitingForStart)) {
                    self.readInput.append(1)
                    waitingForStart = true
                    print(self.readInput)
                }
                if (abs(frequency - Float(lowFrequency)) < lenianceValue && (!waitingForStart)) {
                    self.readInput.append(0)
                    waitingForStart = true
                    print(self.readInput)
                }
            }

            vDSP_destroy_fftsetup(fftSetup)
        }

        do {
            let session = AVAudioSession.sharedInstance()
            try session.setCategory(.playAndRecord, mode: .default, options: [.defaultToSpeaker])
            try session.setActive(true)
            try session.overrideOutputAudioPort(.speaker)
            try microphoneAudioEngine.start()
            
            let listeningTimeout = 15 * frequencyDuration * 3 // 15 bits + in between bits + some extra if delayed
            let timeInterval = stopListeningImmediately ? 0.0 : listeningTimeout
            
            Timer.scheduledTimer(withTimeInterval: timeInterval, repeats: false) { [weak self] _ in
                        guard let self = self else { return }
                        self.microphoneAudioEngine.inputNode.removeTap(onBus: 0)
                        self.microphoneAudioEngine.stop()

                        self.state = readerState.IDLE

                        print("Stopped listening, switching to sending state.")
                    }
        } catch {
            print("Audio session configuration error: \(error)")
        }
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        microphoneAudioEngine.inputNode.removeTap(onBus: 0)
        microphoneAudioEngine.stop()
    }
    
}


// extensions to manage the data table:

extension NodePageViewController: UITableViewDelegate {}

extension NodePageViewController: UITableViewDataSource {
    
    // setting number of entries in the table
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        tableView.separatorStyle = .none
        return sensorReadings.count
    }
    
    // defining the content of each cell
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "nodeDataCell", for: indexPath)
        cell.backgroundColor = .clear
        cell.selectionStyle = .none
        cell.contentView.subviews.forEach { $0.removeFromSuperview() }
            
        let containerView = UIView(frame: CGRect(x: 10, y: 5, width: tableView.frame.width - 20, height: 50))
        containerView.backgroundColor = .white
        containerView.layer.cornerRadius = 10
        containerView.layer.masksToBounds = true
            
        let label = UILabel(frame: CGRect(x: 10, y: 0, width: containerView.frame.width - 20, height: 50))
        label.text = sensorReadings[indexPath.row].sensorName + ": " + String(sensorReadings[indexPath.row].sensorValue)
        label.textAlignment = .left
            
        containerView.addSubview(label)
        cell.contentView.addSubview(containerView)
            
        return cell
    }
    
    // setting the height of each cell
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return 60
    }
}
