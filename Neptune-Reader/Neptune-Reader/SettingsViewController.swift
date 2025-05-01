//
//  SettingsViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/15/25.
//

import UIKit

class SettingsViewController: UIViewController {

    @IBOutlet weak var bitDurationField: UITextField!
    @IBOutlet weak var highFreqField: UITextField!
    @IBOutlet weak var lowFreqField: UITextField!
    @IBOutlet weak var endFreqField: UITextField!
    @IBOutlet weak var startFreqField: UITextField!
    @IBOutlet weak var fftSizeField: UITextField!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        bitDurationField.returnKeyType = .done
        bitDurationField.delegate = self
        
        highFreqField.returnKeyType = .done
        highFreqField.delegate = self
        
        lowFreqField.returnKeyType = .done
        lowFreqField.delegate = self
        
        startFreqField.returnKeyType = .done
        startFreqField.delegate = self
        
        endFreqField.returnKeyType = .done
        endFreqField.delegate = self
        
        fftSizeField.returnKeyType = .done
        fftSizeField.delegate = self
        
        bitDurationField.text = String(Int(frequencyDuration * 1000))
        highFreqField.text = String(Int(highFrequency))
        lowFreqField.text = String(Int(lowFrequency))
        startFreqField.text = String(Int(startFrequency))
        endFreqField.text = String(Int(endFrequency))
        fftSizeField.text = String(Int(fftSize))
    }
}

extension SettingsViewController: UITextFieldDelegate {
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        textField.resignFirstResponder()
        guard let text = textField.text, let doubleValue = Double(text) else {
                print("Invalid input: Not a valid number")
                return true
            }
            
            if textField == bitDurationField {
                frequencyDuration = doubleValue / 1000
                print("Frequency duration updated to: \(frequencyDuration)")
                
            } else if textField == highFreqField {
                highFrequency = doubleValue
                print("High frequency updated to: \(highFrequency)")
            }
            else if textField == lowFreqField {
                lowFrequency = doubleValue
                print("High frequency updated to: \(lowFrequency)")
            }
            else if textField == startFreqField {
                startFrequency = doubleValue
                print("High frequency updated to: \(startFrequency)")
            }
            else if textField == endFreqField {
                endFrequency = doubleValue
                print("High frequency updated to: \(endFrequency)")
            }
            else if textField == fftSizeField {
                fftSize = Int(doubleValue)
                print("FFT size updated to: \(fftSize)")
            }
        
        return true
    }
}
