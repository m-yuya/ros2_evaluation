// #include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
// #include "std_msgs/String.h"
#include <std_msgs/msg/string.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <time.h>				// clock
#include <unistd.h>				// clock
#include <sys/mman.h>			// mlock
#include <sched.h>				// sched
#include <arpa/inet.h>			// socket

#define EVAL_NUM 120
#define IS_RELIABLE_QOS 0			// 1 means "reliable"", 0 means "best effort""

static const rmw_qos_profile_t rmw_qos_profile_reliable = {
  RMW_QOS_POLICY_KEEP_ALL_HISTORY,
  5,
  RMW_QOS_POLICY_RELIABLE,
  RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY
};

static const rmw_qos_profile_t rmw_qos_profile_best_effort = {
  RMW_QOS_POLICY_KEEP_LAST_HISTORY,
  1,
  RMW_QOS_POLICY_BEST_EFFORT,
  RMW_QOS_POLICY_VOLATILE_DURABILITY
};

int i, initializer=0;

struct timespec tp1;			// for clock

FILE *fp;						// for file io

struct sockaddr_in addr;		// for socket
struct sockaddr_in client;
socklen_t client_size;
int sock0, sock;

//void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const std_msgs::msg::String::SharedPtr msg){

  // printf("subscribe: [%s]\n", receiver.data.c_str());
  //  printf("subscribe \n");

  if( initializer == 0 ) {
 	// Initialize
 	char init_num_char = *( msg->data.c_str());
 	char *init_num_pt = &init_num_char;
 	initializer = atoi(init_num_pt);
	//	printf("initializer : %d \n", initializer);
 	if ( initializer == 1 ){
 	  printf("start evaluation as a server \n");
 	}
  }
  if ( initializer == 1 ){
 	/* write(ソケット,"文字",文字数) */
	//	printf("write \n");
 	write(sock, "x", 1);
  }
}

int set_bind_listen_accept_socket(){

  /* ソケットの作成 */
  printf("set \n");
  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  if( sock0<0 ){
  	perror("socket");
  	return 1;
  }

  /* ソケットの設定 */
  addr.sin_family = AF_INET;
  addr.sin_port = htons(12345);
  addr.sin_addr.s_addr = INADDR_ANY;

  // portがTIME_WAIT状態でも接続する
  const int one = 1;
  setsockopt(sock0, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));

  printf("bind \n");
  if (  bind(sock0, (struct sockaddr *)&addr, sizeof(addr))<0 ){
  	perror("bind");
  	return 1;
  }

  /* TCPクライアントからの接続要求を待てる状態にする */
  printf("listen \n");
  if(   listen(sock0, 5)<0 ){
  	perror("listen");
  	return 1;
  }

  client_size = sizeof(client);
  printf("accept \n");
  printf("waiting for talker_client... \n");

  //クライアントから通信があるまで待機
  sock = accept(sock0, (struct sockaddr *)&client, &client_size);
  if( sock<0 ){
	perror("accept");
	return 1;
  }

  printf("accepted connection from %s, port=%d\n",
		 inet_ntoa(client.sin_addr), ntohs(client.sin_port));

  return 0;
}


// int main(int argc, char **argv)
int main(int argc, char * argv[])
{
  mlockall(MCL_FUTURE);
  
  usleep(1000);					// avoid race condition
  sched_param  pri = {94}; 
  if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) {
  	perror("sched_setattr");
  	exit(EXIT_FAILURE);
  }

  //   ros::init(argc, argv, "listener");
  rclcpp::init(argc, argv);

  //   ros::NodeHandle n;
  auto node = rclcpp::Node::make_shared("listener");

  // QoS settings
  rmw_qos_profile_t custom_qos_profile;
  if( IS_RELIABLE_QOS == 1){
	custom_qos_profile = rmw_qos_profile_reliable;
  }else{
	custom_qos_profile = rmw_qos_profile_best_effort;
  }
  
  //   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  auto sub = node->create_subscription<std_msgs::msg::String>("chatter", chatterCallback,  custom_qos_profile);

  // wait for establishing socket
  if ( set_bind_listen_accept_socket() == 1 ){
  	perror("set_bind_listen_accept_socket");
  	return 1;
  }
 
  //   ros::spin();
  rclcpp::spin(node);

  return 0;
}
