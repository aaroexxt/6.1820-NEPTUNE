//
//  HelperFunctions.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/12/25.
//

import UIKit
import AVFoundation
import Accelerate
import Foundation
import CoreMotion

/*
 #### IMPORTANT VARIABLES ####
 */

var startFrequency: Double = 10000.0
var endFrequency: Double = 12000.0
var lowFrequency: Double = 9000.0
var highFrequency: Double = 11000.0

var frequencyDuration = 0.13 // 500ms
let lenianceValue: Float = 20.0

var fftSize = 512  // 1024
let sampleRate: Double = 44100.0

let dataRequestMessageBits: [String: String] = [
    "Gyro X": "00000001",
    "Gyro Y": "00000010",
    "Gyro Z": "00000011",
    "Acceleration X": "00000100",
    "Acceleration Y": "00000101",
    "Acceleration Z": "00000110",
    "Magnetic Field X": "00000111",
    "Magnetic Field Y": "00001000",
    "Magnetic Field Z": "00001001",
    "Orientation X": "00001010",
    "Orientation Y": "00001011",
    "Orientation Z": "00001100",
    "Temperature": "00001101",
    "Toggle LEDs": "00001110",
    "Get Battery Voltage": "00001111",
    "Get Battery Percent": "00010000",
]

let dataScales: [String: [Double]] = [
    "Gyro X": [-200.0, 200.0],
    "Gyro Y": [-200.0, 200.0],
    "Gyro Z": [-200.0, 200.0],
    "Acceleration X": [-5.0, 5.0],
    "Acceleration Y": [-5.0, 5.0],
    "Acceleration Z": [-5.0, 5.0],
    "Magnetic Field X": [-500.0, 500.0],
    "Magnetic Field Y": [-500.0, 500.0],
    "Magnetic Field Z": [-500.0, 500.0],
    "Orientation X": [0.0, 360.0],
    "Orientation Y": [0.0, 360.0],
    "Orientation Z": [0.0, 360.0],
    "Temperature": [0.0, 40.0],
]

/*
 #### MESSAGES ####
 */

struct Message {
    var id: [Int] // length 2 for a 2-bit message
    var messageBits: [Int] // 8 bits
    var hammingBits: [Int] // 4 bits
    var overallParity: Int // 1 bit
}

/*
 Given an ID and message bits, return an instance of the Message struct (figures out hamming bits & parity bit).
 */
func constructMessage(id: String, messageBits: String) -> Message {
    let msgId = id.map { Int(String($0))! }
    let msgMessageBits = messageBits.map { Int(String($0))! }
    let idAndMessage = msgId + msgMessageBits
    let msgHammingBits = getHammingBits(idAndMsgBits: idAndMessage)
    
    var messageFull = msgId + msgMessageBits[0...3]
    messageFull += [msgHammingBits[0]]
    messageFull += msgMessageBits[4...6]
    messageFull += [msgHammingBits[1]]
    messageFull += [msgMessageBits[7]]
    messageFull += msgHammingBits[2...3]
    
    let msgOverallParity = getOverallParityBit(allOtherBits: messageFull)
    return Message(id: msgId, messageBits: msgMessageBits, hammingBits: msgHammingBits, overallParity: msgOverallParity)
}

/*
 Helper function to get the 4 hamming bits from 10 bits (2 id bits + 8 message bits)
 */
func getHammingBits(idAndMsgBits: [Int]) -> [Int] {
    var hammingBits = [Int](repeating: 0, count: 4)

    for i in 0..<4 {
        let parityPos = 1 << i // 1, 2, 4, 8
        var parity = 0

        for j in 1...idAndMsgBits.count {
            if (j & parityPos) != 0 {
                parity ^= idAndMsgBits[j - 1]
            }
        }
        hammingBits[i] = parity
    }

    return hammingBits
}

/*
 Helper function to get one parity bit from a list of bits
 */
func getOverallParityBit(allOtherBits: [Int]) -> Int {
    var parity = 0
        for bit in allOtherBits {
            parity ^= Int(bit)
        }
        return parity
}



/*
 #### PLAYING SPEAKER TONES/SENDING MESSAGES ####
 */

enum readerState {
    case LISTENING // using mic to recieve messages
    case SENDING // using speaker to send out a message
    case IDLE // paused
}

/*
 Helper class just needed to keep track of speaker source nodes
 */
class AudioWrapper {
    var sourceNode: AVAudioSourceNode
    init(sourceNode: AVAudioSourceNode) {
        self.sourceNode = sourceNode
    }
}

/*
 Given a Message, construct its order of bits and play its tones on the speaker
 */
func sendMessage(msgToSend: Message, speakerAudioEngine: AVAudioEngine, audioWrapper: AudioWrapper) {
    Thread.sleep(forTimeInterval: frequencyDuration) // short delay so it doesn't overlap with the message its responding to
    
    // creating ordered bits
    var combinedMessage = msgToSend.id + msgToSend.messageBits[0...3]
    combinedMessage += [msgToSend.hammingBits[0]]
    combinedMessage += msgToSend.messageBits[4...6]
    combinedMessage += [msgToSend.hammingBits[1]]
    combinedMessage += [msgToSend.messageBits[7]]
    combinedMessage += msgToSend.hammingBits[2...3]
    combinedMessage += [msgToSend.overallParity]
    
    print("sending message: ", combinedMessage)
    
    // play each of the bits in order (interspersed with start tone)
    for i in 0...(combinedMessage.count-1) {
        playTone(freq: startFrequency, speakerAudioEngine: speakerAudioEngine, audioWrapper: audioWrapper)
        Thread.sleep(forTimeInterval: frequencyDuration)
        stopTone(audioWrapper: audioWrapper, speakerAudioEngine: speakerAudioEngine)
        
        let freqToPlay = (combinedMessage[combinedMessage.index(combinedMessage.startIndex, offsetBy: i)] == 1) ? highFrequency : lowFrequency
        playTone(freq: freqToPlay, speakerAudioEngine: speakerAudioEngine, audioWrapper: audioWrapper)
        Thread.sleep(forTimeInterval: frequencyDuration)
        stopTone(audioWrapper: audioWrapper, speakerAudioEngine: speakerAudioEngine)
    }
    playTone(freq: endFrequency, speakerAudioEngine: speakerAudioEngine, audioWrapper: audioWrapper)
    Thread.sleep(forTimeInterval: frequencyDuration)
    stopTone(audioWrapper: audioWrapper, speakerAudioEngine: speakerAudioEngine)
}

/*
 Given a frequency, play its pure tone (sine wave)
 */
func playTone(freq: Double, speakerAudioEngine: AVAudioEngine, audioWrapper: AudioWrapper) {
    let mainMixer = speakerAudioEngine.mainMixerNode
    let format = mainMixer.outputFormat(forBus: 0)
    let sampleRate = format.sampleRate // wasnt let before
    var theta: Double = 0.0

    let newSourceNode = AVAudioSourceNode { _, _, frameCount, audioBufferList -> OSStatus in
        let ablPointer = UnsafeMutableAudioBufferListPointer(audioBufferList)
        let thetaIncrement = 2.0 * Double.pi * freq / sampleRate

        for frame in 0..<Int(frameCount) {
            let amplitude: Float32 = 15.0 // max 1.0 (15 for very loud)
            let sampleVal = amplitude * Float32(sin(theta))
            theta += thetaIncrement
            if theta > 2.0 * Double.pi {
                theta -= 2.0 * Double.pi
            }

            for buffer in ablPointer {
                let buf = buffer.mData?.assumingMemoryBound(to: Float32.self)
                buf?[frame] = sampleVal
            }
        }

        return noErr
    }
    
    audioWrapper.sourceNode = newSourceNode
    speakerAudioEngine.attach(audioWrapper.sourceNode)
    speakerAudioEngine.connect(audioWrapper.sourceNode, to: mainMixer, format: format)

    do {
        try speakerAudioEngine.start()
    } catch {
        print("Failed to start AVAudioEngine: \(error.localizedDescription)")
    }
}

/*
 Stop all tones playing on the speaker
 */
func stopTone(audioWrapper: AudioWrapper, speakerAudioEngine: AVAudioEngine) {
    speakerAudioEngine.detach(audioWrapper.sourceNode)
}
