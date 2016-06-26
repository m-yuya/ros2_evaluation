# ros2_evaluation

## Evaluation method
In every data size, a publisher-node sends 100 messages per experimet with 10 Hz.
Each simple string message is transported in order.
Using 100 measurements in each data size, we prepare medians and boxplots.
To clarify this process, we make source code for evaluation open and have added its url to references of our paper.

In the remote cases, to avoid time synchronization issues, the experiment adopts simple socket communication that routes through neither ROS1 nor ROS2.
Machine1 transmits data through ROS1 or ROS2, and receives short data through socket communication.
In the adopted method, evaluation halts when messages do not reach a subscriber-node in the cases with, for example, the  best-effort policy, because a publisher-node must wait until a subscriber-node replies during each publish event.
We estimate end-to-end latencies by subtracting preliminarily evaluated socket communication time.
Using socket communication, the communication latencies between ROS1 and ROS2 can be evaluated respectively. 
However, dividing round-trip latency in half cannot evaluate them and does not be used for this evaluation.
The following figure shows the node-graph for evaluation of communication from ROS1 to ROS2 with socket communication and a ros\_bridge in remote cases.
For page constraint, we could not have added the above explanation and following figure.
Hence, we make source code for evaluation open and have added its url to references of our paper.

![evaluation method in (3-a)](https://raw.githubusercontent.com/m-yuya/ros1_evaluation/images/images/eval_method.png)

![evaluation situation](https://raw.githubusercontent.com/m-yuya/ros1_evaluation/images/images/eval_situation.png)
