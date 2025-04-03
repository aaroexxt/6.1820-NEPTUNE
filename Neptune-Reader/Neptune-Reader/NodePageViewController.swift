//
//  NodePageViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/2/25.
//

import UIKit

class NodePageViewController: UIViewController {
    
    @IBOutlet weak var nodeLabel: UILabel!
    @IBOutlet var dataTable: UITableView!
    
    struct SensorReading {
        var sensorName: String
        var sensorValue: Double
    }

    // initializing a list of all the sensors on the node
    var sensorReadings: [SensorReading] = [
        SensorReading(sensorName: "Temperature", sensorValue: 0.0),
        SensorReading(sensorName: "Acceleration", sensorValue: 0.0),
        SensorReading(sensorName: "Magnetic Field", sensorValue: 0.0)
    ]
    
    var selectedNode: String? // the name of this node (passed in from viewController)

    override func viewDidLoad() {
        super.viewDidLoad()

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
        
        // get initial sensor readings
        refreshButtonTapped(UIButton())
    }
    
    @IBAction func refreshButtonTapped(_ sender: UIButton) {
        print("Refreshing sensor readings...")
        // for now filled with random values
        // todo: fill in sensorReadings with real data
        for i in 0...(sensorReadings.count-1) {
            let randomValue = Double.random(in: 0.0...100.0).rounded() / 100 + Double.random(in: 0.0...100.0).rounded()
            sensorReadings[i].sensorValue = randomValue
        }
        dataTable.reloadData()
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
