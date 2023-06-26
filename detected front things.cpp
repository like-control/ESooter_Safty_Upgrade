#include <iostream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <cmath>

#include "uart.h"

#include <alsa/asoundlib.h>

#include <ctime>

 

const int WIDTH = 640; // 프레임의 가로 크기

const int HEIGHT = 480; // 프레임의 세로 크기

const int FPS = 30; // 프레임 레이트

float MIN_WIDTH = 80.0f; // 감지할 최소 가로 크기 (단위-픽셀)

float MIN_HEIGHT = 10.0f; // 감지할 최소 세로 크기 (단위-픽셀)

const float MAX_DISTANCE = 6000; // 감지할 최대 거리 (단위-mm)

 

clock_t start;

clock_t finish;

 

const char* device = "plughw:2,0"; // 사용할 오디오 장치

const char* file = "alert.wav"; // 재생할 WAV 파일 경로

 

int obstacle[640][480];

 

void Start_Brake(Uart u, float S_Speed)

{

start = clock();

 

char S_data[5];

 

sprintf(S_data, "$%d#", (int)S_Speed);

u.sendUart(S_data, strlen(S_data));

usleep(1000);

 

finish = clock();

 

int duration = (double)(finish - start);

std::cout << duration << "ms" << std::endl;

}

 

float Cal_Distance(double distance)

{

int num;

 

distance /= 1000;

std::cout << "DISTANCE/1000 : " << distance <<"\n";

num = 2 * 0.8 * 9.8 * distance;

std::cout << "num : " << num <<"\n";

std::cout << "sqrt num : " << std::sqrt(num) <<"\n";

return std::sqrt(num);

}

 

void Re_size(float distance)

{

if(500 <= distance && distance < 1500)

{

MIN_WIDTH = 490.0f;

MIN_HEIGHT = 270.0f;

}

else if(1500 <= distance && distance < 3000)

{

MIN_WIDTH = 240.0f;

MIN_HEIGHT = 90.0f;

}

else if(3000 <= distance && distance < 4500)

{

MIN_WIDTH = 120.0f;

MIN_HEIGHT = 30.0f;

}

else if(4500 <= distance && distance < 6000)

{

MIN_WIDTH = 60.0f;

MIN_HEIGHT = 10.0f;

}

else if(6000 <= distance)

{

MIN_WIDTH = 30.0f;

MIN_HEIGHT = 3.0f;

}

}

 

int main(int argc, char * argv[])

{

float S_Speed;

float Speed;

bool is_one_exist = false;

double distance;

 

Uart u;

// 윤곽선 포인트 저장 생성자

std::vector<std::vector<cv::Point>> contours;

// 윤곽선 hierarchy 정보 저장 생성자

std::vector<cv::Vec4i> hierarchy;

 

// ******************** RealSense D455 초기화 ****************************************

// 데이터 처리를 위한 파이프라인 생성

rs2::pipeline pipe;

// 스트림 생성

rs2::config cfg;

// (활성화할 스트림 유형, 너비, 높이, 픽셀 형식, 프레임속도)

// 스트림 활성화 픽셀 형식 => (16비트 선형 깊이 값. 깊이는 미터가 깊이 스케일 * 픽셀 값)

cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);

// 파이프라인 실행

pipe.start(cfg);

 

// ************************ OpenCV 초기화 *********************

// depth 프레임을 opencv 이미지로 변환한 것을 저장하기 위한 생성자

cv::Mat depth_image;

// 이진화 데이터를 저장하기 위한 생성자

cv::Mat threshold_image; //

cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE); //

 

// 행렬을 생성하고 0으로 초기화합니다.

cv::Mat roi_mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, cv::Scalar(0));

// 사다리꼴을 구성하는 좌표를 정의합니다.

std::vector<cv::Point> roi_points = { cv::Point(215, 300), cv::Point(425, 300), cv::Point(635, 475), cv::Point(5, 475) };

// fillConvexPoly 함수를 사용하여 사다리꼴 영역을 1로 설정합니다.

cv::fillConvexPoly(roi_mat, roi_points, cv::Scalar(1));

 

// 행렬을 생성하고 0으로 초기화합니다.

cv::Mat res_mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, cv::Scalar(0));

 

// ************************** 메인루프 *****************************

while (true) {

// ********** RealSense D455에서 Depth 프레임을 가져옴 ****

// frame 가져온 후 변수 frames에 저장

rs2::frameset frames = pipe.wait_for_frames();

// frames의 깊이 정보 가져온 후 변수 depth_frame에 저장

rs2::depth_frame depth_frame = frames.get_depth_frame();

if (!depth_frame) {

continue;

}

 

// Depth 프레임을 OpenCV 이미지로 변환

depth_image = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

 

// ********** Depth 이미지를 이진화하여 장애물 영역을 구함 **********

// (입력 배열, 출력 배열, 임계치, 최대값, 임계값 유형)

// 임계치 최대 거리, 임계값 유형 : 이진화

cv::threshold(depth_image, threshold_image, MAX_DISTANCE, 255, cv::THRESH_TOZERO_INV);

// 입력 배열을 컬러로 변환

threshold_image.convertTo(threshold_image, CV_8UC1);

// 이진화 이미지의 윤곽선을 표현하는 포인트 요소를 벡터로 표현하여(contours) 가지고있음

cv::findContours(threshold_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

 

// ********** 감지된 모든 장애물에 대해서 처리 **********

for (size_t i = 0; i < contours.size(); i++) {

// ********** 장애물의 면적을 계산 **********

// 장애물의 픽셀 수 area 변수에 저장

float area = cv::contourArea(contours[i]);

 

// 윤곽선을 감싸는 최소한의 사각형 구하기

cv::Rect rect = cv::boundingRect(contours[i]);

 

// 장애물의 중심점을 계산

cv::Moments moments = cv::moments(contours[i]);

float x = moments.m10 / moments.m00;

float y = moments.m01 / moments.m00;

 

// 중심점과 Depth 값으로 장애물과 카메라의 거리를 계산

//

distance = depth_frame.get_distance(x, y) * 1000;

if (distance > MAX_DISTANCE)

{

continue;

}

 

Re_size(distance);

 

// 최소 사이즈보다 작은 경우

if (rect.width < MIN_WIDTH || rect.height < MIN_HEIGHT) {

// 최소 사각형 크기만큼 이진데이터 0으로 변경(있는 장애물로 간주 x)

for (int i = rect.y; i < rect.y + rect.height; i++) {

for (int j = rect.x; j < rect.x + rect.width; j++) {

depth_image.at<uchar>(i, j) = 0;

}

}

// 다음 장애물로 넘어가기

continue;

}

 

// 원본 이미지에 사각형 그리기

cv::rectangle(depth_image, rect, cv::Scalar(255, 0, 0), 2);

}

 

// mat과 grayscale_image의 각 요소의 값을 곱하여 res_mat에 저장합니다.

for (int y = 0; y < roi_mat.rows; y++) {

for (int x = 0; x < roi_mat.cols; x++) {

res_mat.at<uint8_t>(y, x) = roi_mat.at<uint8_t>(y, x) * depth_image.at<uint8_t>(y, x);

}

}

 

// res_mat의 모든 요소를 탐색하면서 1이 존재하는지 검사합니다.

for (int y = 0; y < res_mat.rows; y++) {

for (int x = 0; x < res_mat.cols; x++) {

if (res_mat.at<uint8_t>(y, x) == 1) {

is_one_exist = true;

break;

}

}

}

 

// 1이 존재하면 경고 메시지를 출력합니다.

if (is_one_exist) {

std::cout << "Warning: res_mat contains at least one 1." << std::endl;

std::cout << "DISTANCE : " << distance <<"\n";

S_Speed = Cal_Distance(distance);

std::cout << "S_Speed : " << S_Speed <<"\n";

Start_Brake(u, S_Speed);

 

// ALSA 핸들 초기화

snd_pcm_t *pcm_handle;

if (snd_pcm_open(&pcm_handle, device, SND_PCM_STREAM_PLAYBACK, 0) < 0) {

printf("오디오 장치를 열 수 없습니다.\n");

return -1;

}

 

// WAV 파일 열기

FILE *wav_file = fopen(file, "rb");

if (!wav_file) {

printf("WAV 파일을 열 수 없습니다.\n");

return -1;

}

 

// WAV 헤더 읽기

char header[44];

if (fread(header, sizeof(header), 1, wav_file) != 1) {

printf("WAV 헤더를 읽을 수 없습니다.\n");

return -1;

}

 

// PCM 파라미터 설정

unsigned int sample_rate = *(unsigned int*)&header[24];

unsigned int channels = *(unsigned short*)&header[22];

snd_pcm_set_params(pcm_handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,

channels, sample_rate, 1, 500000);

 

// 소리 데이터 재생

const int buffer_size = 1024;

char buffer[buffer_size];

int read_size;

while ((read_size = fread(buffer, 1, buffer_size, wav_file)) > 0) {

snd_pcm_writei(pcm_handle, buffer, read_size / 4); // 2바이트(16비트) 샘플이므로 나누기 4

}

 

// 리소스 정리

snd_pcm_drain(pcm_handle);

snd_pcm_close(pcm_handle);

fclose(wav_file);

is_one_exist = false;

}

else{

std::cout << "---No WARNING---" << std::endl;

Start_Brake(u, 35);

}

 

// ********** Depth 이미지를 화면에 출력 **********

// 원본 이미지에 사다리꼴 그리기

std::vector<std::vector<cv::Point>> contour = { roi_points };

cv::polylines(depth_image, contour, true, cv::Scalar(0, 255, 0), 2);

cv::imshow("Depth Image", depth_image);

if (cv::waitKey(1) == 27) {

//closeUart();

break;

}

}

}
