//
//  ViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/1/25.
//

import UIKit
import AVFoundation
import Accelerate

class ViewController: UIViewController {
    
    @IBOutlet var tableView: UITableView!
    
    struct Node {
        var name: String
        var isConnected: Bool
    }
    
    var speakerAudioEngine = AVAudioEngine()
    var sourceNode: AVAudioSourceNode!
    
    var microphoneAudioEngine = AVAudioEngine()
    
    var readInput: [Int] = []
    var audioWrapper: AudioWrapper!
    
    var state: readerState = readerState.IDLE

    var nodes: [Node] = [
        Node(name: "Node 1", isConnected: false),
        Node(name: "Node 2", isConnected: false),
        Node(name: "Node 3", isConnected: false),
    ]
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        let session = AVAudioSession.sharedInstance()
            do {
                try session.setCategory(.playAndRecord, mode: .measurement, options: [.mixWithOthers, .defaultToSpeaker])
                try session.setActive(true)
                try session.overrideOutputAudioPort(.speaker)
            } catch {
                print("Failed to configure AVAudioSession: \(error)")
            }
        
        self.sourceNode = AVAudioSourceNode { _, _, _, _ in return noErr }
        self.audioWrapper = AudioWrapper(sourceNode: self.sourceNode)
        
        // setup UI table
        tableView.delegate = self
        tableView.dataSource = self
        
        // get rid of table's whitespace
        let v = UIView()
        v.backgroundColor = .clear
        tableView.tableFooterView = v
        tableView.backgroundColor = .clear
        
        requestMicrophoneAccess(stopListeningImmediately: true)
    }
    
    /*
     Runs on messages received from the microphone. Converts the list of bit values into its node id bits, message bits, hamming bits, and parity bit.
     Checks the message and sends a response accordingly.
     */
    func decodeMessage(msg: [Int], speakerAudioEngine: inout AVAudioEngine, sourceNode: inout AVAudioSourceNode) {
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
        print("Is valid? ", isValid)
        
        if(isValid) {
            let stringID = nodeID.map { String($0) }.joined()
            let stringMessage = message.map { String($0) }.joined()
            
            // message that node 1 is alive received
            if(stringID == "01" && stringMessage == "01010101") {
                nodes[0].isConnected = true
                nodes[1].isConnected = false
                DispatchQueue.main.async {
                    self.tableView.reloadData()
                }
                pauseLoop()
            }
            
            // message that node 2 is alive received
            if(stringID == "10" && stringMessage == "01010101") {
                nodes[0].isConnected = false
                nodes[1].isConnected = true
                DispatchQueue.main.async {
                    self.tableView.reloadData()
                }
                pauseLoop()
            }
            
            
        }
    }
    
    /*
     When user asks for a refresh, sends out a message asking for node connections. From the decoded response updates UI.
     */
    @IBAction func refreshButtonTapped(_ sender: UIButton) {
        if(state == readerState.IDLE) {
            print("starting search!")
            self.state = readerState.SENDING
            self.startLoop()
        }
        else {
            print("pausing search!")
            self.pauseLoop()
        }
        
        // tableView.reloadData()
    }
    
    func startLoop() {
            print("state: ", self.state)
            if(self.state == readerState.SENDING) {
                self.readInput = [] // reset the buffer
                
                // asking node 1 if it's connected
                let askMessage = constructMessage(id: "01", messageBits: "00000000")
                sendMessage(msgToSend: askMessage, speakerAudioEngine: self.speakerAudioEngine, audioWrapper: self.audioWrapper)
                
                self.state = readerState.LISTENING
                requestMicrophoneAccess(stopListeningImmediately: false) // start listening
            }
    }
    
    func pauseLoop() {
        self.state = readerState.IDLE
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
                print("freq: ", frequency)
                
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

                        if(stopListeningImmediately || self.state == readerState.IDLE) {
                            self.state = readerState.IDLE
                        }
                        else {
                            self.state = readerState.SENDING
                            self.startLoop()
                        }

                        print("Stopped listening, switching to sending state.")
                    }
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
    }

}


// extensions to manage the UI table:

extension ViewController: UITableViewDelegate {
    
    // when a row is clicked, navigate to it's page if the node is connected
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        if(nodes[indexPath.row].isConnected) {
            let storyboard = UIStoryboard(name: "Main", bundle: nil)
            if let nodeVC = storyboard.instantiateViewController(withIdentifier: "nodePage") as? NodePageViewController {
                nodeVC.selectedNode = nodes[indexPath.row].name // setting title of node screen
                    navigationController?.pushViewController(nodeVC, animated: true)
                }
            }
    }
}

extension ViewController: UITableViewDataSource {
    
    // setting number of entries in the table
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        tableView.separatorStyle = .none
        return nodes.count
    }
    
    // defining the content of each cell
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "cell1", for: indexPath)
        cell.backgroundColor = .clear
        cell.selectionStyle = .none
        cell.contentView.backgroundColor = .clear

        // Remove previous container views (important when reusing cells)
        cell.contentView.subviews.forEach { $0.removeFromSuperview() }

        // Set up containerView
        let containerView = UIView()
        containerView.translatesAutoresizingMaskIntoConstraints = false
        containerView.layer.cornerRadius = 10

        if nodes[indexPath.row].isConnected {
            containerView.backgroundColor = UIColor(red: 0.78, green: 0.96, blue: 0.76, alpha: 1.0) // light green
        } else {
            containerView.backgroundColor = UIColor(red: 0.27, green: 0.48, blue: 0.62, alpha: 1.0) // bluish
        }

        // Add container to cell
        cell.contentView.addSubview(containerView)

        // Add label inside containerView
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.text = nodes[indexPath.row].name + " - " + (nodes[indexPath.row].isConnected ? "Connected" : "Offline")
        label.textColor = UIColor(red: 0.11, green: 0.21, blue: 0.34, alpha: 1.0)
        containerView.addSubview(label)

        // Set Auto Layout constraints
        NSLayoutConstraint.activate([
            // Padding around containerView inside the cell
            containerView.topAnchor.constraint(equalTo: cell.contentView.topAnchor, constant: 5),
            containerView.bottomAnchor.constraint(equalTo: cell.contentView.bottomAnchor, constant: -5),
            containerView.leadingAnchor.constraint(equalTo: cell.contentView.leadingAnchor, constant: 10),
            containerView.trailingAnchor.constraint(equalTo: cell.contentView.trailingAnchor, constant: -10),

            // Padding around label inside the containerView
            label.topAnchor.constraint(equalTo: containerView.topAnchor, constant: 20),
            label.bottomAnchor.constraint(equalTo: containerView.bottomAnchor, constant: -20),
            label.leadingAnchor.constraint(equalTo: containerView.leadingAnchor, constant: 20),
            label.trailingAnchor.constraint(equalTo: containerView.trailingAnchor, constant: -20),
        ])

        return cell
    }


    
    // setting the height of each cell
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return 65
    }
}
