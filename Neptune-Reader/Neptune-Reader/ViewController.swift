//
//  ViewController.swift
//  Neptune-Reader
//
//  Created by Nora Bulovic on 4/1/25.
//

import UIKit

class ViewController: UIViewController {
    
    @IBOutlet var tableView: UITableView!
    
    struct Node {
        var name: String
        var isConnected: Bool
    }
    
    // initializing list of all nodes
    var nodes: [Node] = [
        Node(name: "Node 1", isConnected: false),
        Node(name: "Node 2", isConnected: false),
        Node(name: "Node 3", isConnected: false),
    ]
    

    override func viewDidLoad() {
        super.viewDidLoad()
        
        // setup UI table
        tableView.delegate = self
        tableView.dataSource = self
        
        // get rid of table's whitespace
        let v = UIView()
        v.backgroundColor = .clear
        tableView.tableFooterView = v
        tableView.backgroundColor = .clear
    }
    
    // re-check which nodes are connected
    @IBAction func refreshButtonTapped(_ sender: UIButton) {
        // for now chose random nodes to be connected
        // todo: change this into actual data
        for i in 0...(nodes.count-1) {
            nodes[i].isConnected = Bool.random()
        }
        tableView.reloadData()
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

        let displayText = nodes[indexPath.row].name + " - " + (nodes[indexPath.row].isConnected ? "Connected" : "Offline")
        cell.textLabel?.text = displayText
        
        let containerView = UIView(frame: CGRect(x: 10, y: 5, width: tableView.frame.width - 20, height: 50))
        if(nodes[indexPath.row].isConnected) {
            containerView.backgroundColor = UIColor(red: 0.78, green: 0.96, blue: 0.76, alpha: 1.0) // green when connected
        }
        else {
            containerView.backgroundColor = UIColor.systemGray
        }
        containerView.layer.cornerRadius = 10
            
        cell.contentView.subviews.forEach { $0.removeFromSuperview() }
        cell.contentView.addSubview(containerView)
        
        return cell
    }
    
    // setting the height of each cell
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return 60
    }
}
