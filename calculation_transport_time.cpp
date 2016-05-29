#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>				// file io
#include <time.h>				// clock
#include <unistd.h>				// clock

#define LIMIT_NUM 1000

int calculation(std::string pubtime_filename, std::string subtime_filename, std::string transtime_filename){
  int i, n=0;
  int init_num_int;
	
  double publish_time[LIMIT_NUM];
  double subscribe_time[LIMIT_NUM];
  double transport_time[LIMIT_NUM];

  FILE *fp;
	
  // publish_time.txtの読み込み

  fp = fopen(pubtime_filename.c_str(), "r");
  if(fp == NULL) {
	printf("unable to read pub_time \n");
	return 0;
  }
 
  /*  ファイルが終わりでない　「かつ」　配列を飛び出さないうちは，読み込みを続ける  */
  while ( ! feof(fp) && n < LIMIT_NUM) {
	fscanf(fp, "%lf", &(publish_time[n]));
	n++;
  }
  fclose(fp);
  n = n-1; /* 上のwhileループでは，EOFの行を余分に読み込んでいるので，実際のデータ数は一つ少ない． */
 
  /*  画面に表示  */
  for(i=0; i<n; i++) {
	// printf("%lf\n", publish_time[i]);
  }
  printf("------------------\n");

	
  // subscribe_time.txtの読み込み

  fp = fopen(subtime_filename.c_str(), "r");
  if(fp == NULL) {
	printf("unabale to read sub_time \n");
	return 0;
  }
	  
  fscanf(fp, "%d", &(init_num_int));
  printf("%d\n",init_num_int);
  n = 0;
  while ( ! feof(fp) && n < LIMIT_NUM) {
	fscanf(fp, "%lf", &(subscribe_time[n]));
	n++;
  }
  n = n-1;
  /*  画面に表示  */
  for(i=0; i<n; i++) {
	// printf("%lf\n", subscribe_time[i]);
  }


  // 通信時間を計算し、transport_time.txtへ出力

  if((fp = fopen(transtime_filename.c_str(), "w")) != NULL){
	for(i=init_num_int; i<n; i++){
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

}


int main()
{
  calculation("./evaluation/publish_time/publish_time_256byte.txt", "./evaluation/subscribe_time/subscribe_time_256byte.txt", "./evaluation/transport_time/transport_time_256byte.txt");

  calculation("./evaluation/publish_time/publish_time_512byte.txt", "./evaluation/subscribe_time/subscribe_time_512byte.txt", "./evaluation/transport_time/transport_time_512byte.txt");

  calculation("./evaluation/publish_time/publish_time_1Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_1Kbyte.txt", "./evaluation/transport_time/transport_time_1Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_2Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_2Kbyte.txt", "./evaluation/transport_time/transport_time_2Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_4Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_4Kbyte.txt", "./evaluation/transport_time/transport_time_4Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_8Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_8Kbyte.txt", "./evaluation/transport_time/transport_time_8Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_16Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_16Kbyte.txt", "./evaluation/transport_time/transport_time_16Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_32Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_32Kbyte.txt", "./evaluation/transport_time/transport_time_32Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_64Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_64Kbyte.txt", "./evaluation/transport_time/transport_time_64Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_128Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_128Kbyte.txt", "./evaluation/transport_time/transport_time_128Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_256Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_256Kbyte.txt", "./evaluation/transport_time/transport_time_256Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_512Kbyte.txt", "./evaluation/subscribe_time/subscribe_time_512Kbyte.txt", "./evaluation/transport_time/transport_time_512Kbyte.txt");

  calculation("./evaluation/publish_time/publish_time_1Mbyte.txt", "./evaluation/subscribe_time/subscribe_time_1Mbyte.txt", "./evaluation/transport_time/transport_time_1Mbyte.txt");
  
   calculation("./evaluation/publish_time/publish_time_2Mbyte.txt", "./evaluation/subscribe_time/subscribe_time_2Mbyte.txt", "./evaluation/transport_time/transport_time_2Mbyte.txt");

 calculation("./evaluation/publish_time/publish_time_4Mbyte.txt", "./evaluation/subscribe_time/subscribe_time_4Mbyte.txt", "./evaluation/transport_time/transport_time_4Mbyte.txt");
  
  return 0;
}
