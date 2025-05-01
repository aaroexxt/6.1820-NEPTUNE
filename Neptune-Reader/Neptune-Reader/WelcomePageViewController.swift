//
//  WelcomePageViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/9/25.
//

import UIKit

class WelcomePageViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
    }
    
    @IBAction func goToReaderTapped(_ sender: UIButton) {
        print("Navigating to Neptune-Reader...")
        let storyboard = UIStoryboard(name: "Main", bundle: nil)
        if let nodeVC = storyboard.instantiateViewController(withIdentifier: "neptuneReaderHome") as? NodePageViewController {
                navigationController?.pushViewController(nodeVC, animated: true)
            }
    }
    
    @IBAction func goToNodeSimulatorTapped(_ sender: UIButton) {
        print("Navigating to node simulator...")
//        let storyboard = UIStoryboard(name: "Main", bundle: nil)
//        if let nodeVC = storyboard.instantiateViewController(withIdentifier: "nodeSimulator") as? NodePageViewController {
//                navigationController?.pushViewController(nodeVC, animated: true)
//            }
    }
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */

}
