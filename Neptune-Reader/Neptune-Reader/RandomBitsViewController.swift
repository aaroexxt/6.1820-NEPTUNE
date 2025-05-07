//
//  RandomBitsViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 5/6/25.
//

import UIKit
import AVFoundation
import Accelerate

class RandomBitsViewController: UIViewController {
    
    @IBOutlet weak var randomBitsSentLabel: UILabel!
    @IBOutlet weak var totalBitsSentLabel: UILabel!
    
    var speakerAudioEngine = AVAudioEngine()
    var sourceNode: AVAudioSourceNode!
    var audioWrapper: AudioWrapper!
    
    var currentSendingLoopID: UUID?
    
    var state = readerState.IDLE
    
    var allBitsSentString = ""
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.sourceNode = AVAudioSourceNode { _, _, _, _ in return noErr }
        self.audioWrapper = AudioWrapper(sourceNode: self.sourceNode)
        
        totalBitsSent = 0
        
    }
    
    @IBAction func startSendingButtonTapped(_ sender: UIButton) {
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
    
    @IBAction func clearBitStringButtonTapped(_ sender: UIButton) {
        print("clearing sent bits string (and resetting counter)")
        totalBitsSent = 0
        allBitsSentString = ""
        let newLabel = "Sent Bits (" + String(totalBitsSent) + ")"
        DispatchQueue.main.async {
            self.totalBitsSentLabel.text = newLabel
            self.randomBitsSentLabel.text = "None"
        }
        
    }
    
    func startSendingLoop() {
        let newLoopID = UUID()
        currentSendingLoopID = newLoopID
        
        func loop() {
            guard self.state == readerState.SENDING, self.currentSendingLoopID == newLoopID else {
                print("Sending loop cancelled or state changed.")
                return
            }
            
            print("sending loop iteration")
            DispatchQueue.global(qos: .userInitiated).async {
                if(randomModeSendsStartBits) {
                    playTone(freq: startFrequency, speakerAudioEngine: self.speakerAudioEngine, audioWrapper: self.audioWrapper)
                    Thread.sleep(forTimeInterval: frequencyDuration)
                    stopTone(audioWrapper: self.audioWrapper, speakerAudioEngine: self.speakerAudioEngine)
                }
                let highSelected = Bool.random()
                let randomBitFreq = highSelected ? highFrequency : lowFrequency
                playTone(freq: randomBitFreq, speakerAudioEngine: self.speakerAudioEngine, audioWrapper: self.audioWrapper)
                Thread.sleep(forTimeInterval: frequencyDuration)
                stopTone(audioWrapper: self.audioWrapper, speakerAudioEngine: self.speakerAudioEngine)
                totalBitsSent += 1
                let labelText = "Sent Bits (" + String(totalBitsSent) + ")"
                let newBitAsString = highSelected ? "1" : "0"
                self.allBitsSentString += newBitAsString
                DispatchQueue.main.async {
                    self.totalBitsSentLabel.text = String(labelText)
                    self.randomBitsSentLabel.text = String(self.allBitsSentString)
                }
                
                loop()
            }
        }
        
        loop()
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
