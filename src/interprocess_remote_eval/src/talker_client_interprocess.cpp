// #include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
// #include "std_msgs/String.h"
#include <std_msgs/msg/string.hpp>

#include <stdio.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <iostream>				// file io
#include <time.h>				// clock
#include <unistd.h>				// clock
#include <sys/mman.h>			// mlock
#include <sched.h>				// sched
#include <arpa/inet.h>			// socket

#define EVAL_NUM 120
#define LISTENER_IP "192.168.1.70"		
#define PUBLISH_Hz 10
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


int i, count = -1;
double publish_time[EVAL_NUM];
double subscribe_time[EVAL_NUM];
double transport_time[EVAL_NUM];

std::string s, bytedata;

struct timespec tp1;			// for clock

FILE *fp;						// for file io

struct sockaddr_in server;		// for socket
int sock;
char buf[1];
int len;


// file名を引数に取りフアイルの中身を返す関数
std::string read_datafile(std::string message_filename){
  // data_*byte.txtからstd::string bytedataへデータを読み込み 
  
  std::ifstream ifs(message_filename.c_str());
  if(ifs.fail()) {
	std::cerr << "data_*byte.txt do not exist.\n";
	exit(0);
  }

  std::string bytedata;
  getline(ifs, bytedata);

  return bytedata;
}

// EVAL_NUM回、message_filename(data_*byte.txt)をpublishして時刻をoutput_filename(transport_time_*byte.txt)へ出力
int eval_remote_client_ros2(std::string message_filename, std::string output_filename, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub){
  if( -1 < count ){

	auto msg = std::make_shared<std_msgs::msg::String>();	
	std::stringstream ss;
  
	// messageの作成
	ss << 1;					// prefix 1 : listener calls "write()"
	s = ss.str() + bytedata;
	msg->data = s;
  
	// 時刻の記録
	if(clock_gettime(CLOCK_REALTIME,&tp1) < 0){
	  perror("clock_gettime begin");
	  return 0;
	}
	publish_time[count] = (double)tp1.tv_sec + (double)tp1.tv_nsec/ (double)1000000000L;

	// printf("I say: [%s]\n", msg->data.c_str());

	//	printf("publish \n");
	//	chatter_pub.publish(msg);
	chatter_pub->publish(msg);

	/* サーバからデータを受信 */
	//	printf("read \n");
  	len = read(sock, buf, sizeof(buf));
  	if( len <= 0 ){
  	  perror("read");
  	  return 1;
  	}
  	//	printf("%d, %s\n", len, buf);
	
  	if(clock_gettime(CLOCK_REALTIME,&tp1) < 0){
  	  perror("clock_gettime begin");
  	}
  	subscribe_time[count] = (double)tp1.tv_sec + (double)tp1.tv_nsec/ (double)1000000000L;
  
  }
  else if(count == -1){

  	bytedata = read_datafile(message_filename.c_str());
  
  }


  // 通信時間を計算し、transport_time.txtへ出力  
  if( count == EVAL_NUM - 1){

  	if((fp = fopen(output_filename.c_str(), "w")) != NULL){
  	  for(i=0; i<=count; i++){
  		transport_time[i] = subscribe_time[i] - publish_time[i];
  		if(fprintf(fp, "%1.9lf\n", transport_time[i]) < 0){
  		  //書き込みエラー
  		  break;
  		}
  	  }
  	  fclose(fp);
  	}else{
  	  printf("error : can't output file \n");
  	}

  	count = -2;					// initilize for nexy date size
	
  }
  
  count++;
  
  return 0;

}

int set_connect_socket(std::string IP_ADDRESS){

  /* ソケットの作成 */
  printf("set \n");
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if( sock<0 ){

	perror("socket");
	return 1;
  }

  /* 接続先指定用構造体の準備 */
  server.sin_family = AF_INET;
  server.sin_port = htons(12345);
  server.sin_addr.s_addr = inet_addr(IP_ADDRESS.c_str()); // IPアドレスを設定

  /* サーバに接続 */
  printf("connect \n");
  if( connect(sock, (struct sockaddr *)&server, sizeof(server))<0 ){
	perror("connect");
	return 0;
  }

  memset(buf, 0, sizeof(buf));

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
  
  // ros::init(argc, argv, "talker");
  rclcpp::init(argc, argv);

  //  ros::NodeHandle n;  
  auto node = rclcpp::node::Node::make_shared("talker");
  // auto node = std::make_shared<rclcpp::node::Node>("talker"); // test
  // std::shared_ptr<rclcpp::node::Node> node(new rclcpp::node::Node("talker")); // test
  
  // QoS settings
  rmw_qos_profile_t custom_qos_profile;
  if( IS_RELIABLE_QOS == 1){
	custom_qos_profile = rmw_qos_profile_reliable;
  }else{
	custom_qos_profile = rmw_qos_profile_best_effort;
  }
  
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", custom_qos_profile);
  
  // ros::Rate loop_rate(10);
  rclcpp::rate::WallRate loop_rate(PUBLISH_Hz);

  // request establishing socket to IP
  if( set_connect_socket(LISTENER_IP) == 1 ){
	perror("set_connect_socket");
	return 1;
  }
  
  // meanless 20 publish for preparing a secure subscriber
  // 5 and 10 is not enough to conver messages ros1 <-> ros2
  auto msg = std::make_shared<std_msgs::msg::String>();	
  std::stringstream ss;
  ss << 0;						// prefix 0 : listener does not call "write()"
  s = ss.str() + "start";
  msg->data = s;
  i = 1;
  // while (ros::ok())
  while (rclcpp::ok()) {
	if( i++ > 20 ){
	  break;
	}
	//	printf("publish \n");
	chatter_pub->publish(msg);
  	// ros::spinOnce();
	rclcpp::spin_some(node);
  	// loop_rate.sleep();
	loop_rate.sleep();
  }
  
  printf("start evaluation 256byte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_256byte.txt", "./evaluation/transport_time/transport_time_256byte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  usleep(1000000);				// avoid race condition

  printf("start evaluation 512byte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_512byte.txt", "./evaluation/transport_time/transport_time_512byte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 1Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_1Kbyte.txt", "./evaluation/transport_time/transport_time_1Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  printf("start evaluation 2Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_2Kbyte.txt", "./evaluation/transport_time/transport_time_2Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 4Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_4Kbyte.txt", "./evaluation/transport_time/transport_time_4Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 8Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_8Kbyte.txt", "./evaluation/transport_time/transport_time_8Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 16Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_16Kbyte.txt", "./evaluation/transport_time/transport_time_16Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 32Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_32Kbyte.txt", "./evaluation/transport_time/transport_time_32Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 64Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_64Kbyte.txt", "./evaluation/transport_time/transport_time_64Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 128Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_128Kbyte.txt", "./evaluation/transport_time/transport_time_128Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 256Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_256Kbyte.txt", "./evaluation/transport_time/transport_time_256Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 512Kbyte \n");
  while (rclcpp::ok()) {
  	eval_remote_client_ros2("./evaluation/byte_data/data_512Kbyte.txt", "./evaluation/transport_time/transport_time_512Kbyte.txt", chatter_pub);
  	if(count == -1){
  	  printf("break\n");
  	  break;
  	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 1Mbyte \n");
  while (rclcpp::ok()) {
	eval_remote_client_ros2("./evaluation/byte_data/data_1Mbyte.txt", "./evaluation/transport_time/transport_time_1Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  usleep(1000000);
  
  printf("start evaluation 2Mbyte \n");
  while (rclcpp::ok()) {
	eval_remote_client_ros2("./evaluation/byte_data/data_2Mbyte.txt", "./evaluation/transport_time/transport_time_2Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 4Mbyte \n");
  while (rclcpp::ok()) {
	eval_remote_client_ros2("./evaluation/byte_data/data_4Mbyte.txt", "./evaluation/transport_time/transport_time_4Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  // followthrough transactions
  msg->data = "end";
  i = 0;
  while (rclcpp::ok()) {
	chatter_pub->publish(msg);
	if( i++ > 5 ){
	  break;
	}
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  /* socketの終了 */
  close(sock);

  printf("---end evaluation---\n");

  return 0;
}

