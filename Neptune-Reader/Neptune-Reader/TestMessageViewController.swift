//
//  TestMessageViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 5/4/25.
//

import UIKit
import AVFoundation
import Accelerate

class TestMessageViewController: UIViewController {
    @IBOutlet weak var bitsTextField: UITextField!
    
    @IBOutlet weak var sendingFullPacketLabel: UILabel!
    
    @IBOutlet weak var totalBitsSentLabel: UILabel!
    @IBOutlet weak var totalPacketsSentLabel: UILabel!
    @IBOutlet weak var sendTestMessageButton: UIButton!
    var msgToSend: Message = constructMessage(id: "00", messageBits: "00000000")
    
    var speakerAudioEngine = AVAudioEngine()
    var sourceNode: AVAudioSourceNode!
    var audioWrapper: AudioWrapper!
    
    var currentSendingLoopID: UUID?
    
    var state = readerState.IDLE
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        bitsTextField.returnKeyType = .done
        bitsTextField.delegate = self
        
        self.sourceNode = AVAudioSourceNode { _, _, _, _ in return noErr }
        self.audioWrapper = AudioWrapper(sourceNode: self.sourceNode)
        
    }
    
    @IBAction func sendTestMessageButtonTapped(_ sender: UIButton) {
        if self.state == readerState.IDLE {
                print("sending message on loop...")
                sender.setTitle("Pause Sending", for: .normal)
                self.state = readerState.SENDING
                startSendingLoop()
            } else {
                print("pausing...")
                self.state = readerState.IDLE
                self.currentSendingLoopID = nil // cancel current loop
                sender.setTitle("Start Sending", for: .normal)
            }
    }
    
    @IBAction func resetCountersButtonTapped(_ sender: UIButton) {
        print("resetting counters")
        totalBitsSent = 0
        totalPacketsSent = 0
        totalBitsSentLabel.text = String(totalBitsSent)
        totalPacketsSentLabel.text = String(totalPacketsSent)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // stop sending loop
        self.state = readerState.IDLE
        self.currentSendingLoopID = nil
        
        // stop any tones playing
        stopTone(audioWrapper: self.audioWrapper, speakerAudioEngine: self.speakerAudioEngine)
        
        // stop engine
        speakerAudioEngine.stop()
        speakerAudioEngine.reset()
    }

    
}

extension TestMessageViewController: UITextFieldDelegate {
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        textField.resignFirstResponder()
        
        guard let text = textField.text else {
            print("Invalid input: No text entered")
            return true
        }
        
        print("got text field value: ", text)
        let paddedText = text.padding(toLength: 10, withPad: "0", startingAt: 0)
        let id = String(paddedText.prefix(2))
        let messageBitsStartIndex = paddedText.index(paddedText.startIndex, offsetBy: 2)
        let messageBits = String(paddedText[messageBitsStartIndex...].prefix(8))
        print("id: ", id, "messageBits: ", messageBits)
        
        self.msgToSend = constructMessage(id: id, messageBits: messageBits)
        
        var combinedMessage = self.msgToSend.id + self.msgToSend.messageBits[0...3]
        combinedMessage += [self.msgToSend.hammingBits[0]]
        combinedMessage += self.msgToSend.messageBits[4...6]
        combinedMessage += [self.msgToSend.hammingBits[1]]
        combinedMessage += [self.msgToSend.messageBits[7]]
        combinedMessage += self.msgToSend.hammingBits[2...3]
        combinedMessage += [self.msgToSend.overallParity]
        
        print("combined message:", combinedMessage)
        
        sendingFullPacketLabel.text = combinedMessage.map { String($0) }.joined()
        
        return true
    }
    
    func startSendingLoop() {
        let newLoopID = UUID()
        currentSendingLoopID = newLoopID
        let listeningTimeout = 15 * frequencyDuration * 3
        
        func loop() {
            guard self.state == readerState.SENDING, self.currentSendingLoopID == newLoopID else {
                print("Sending loop cancelled or state changed.")
                return
            }
            
            print("sending loop iteration")
            DispatchQueue.global(qos: .userInitiated).async {
                totalPacketsSent += 1
                sendMessage(msgToSend: self.msgToSend, speakerAudioEngine: self.speakerAudioEngine, audioWrapper: self.audioWrapper, uiLabel: self.totalBitsSentLabel, uiLabelPackets: self.totalPacketsSentLabel)
                Thread.sleep(forTimeInterval: listeningTimeout)
                loop()
            }
        }
        
        loop()
    }

}

