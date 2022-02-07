
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include<stack>
using namespace cv;
using namespace std;

const int CircularDistance = 60;				//Բ��Բ֮��ľ���
const int CircularMin = 25;						//Բ������С
const int CircularMax = 35;						//Բ����С��С
const int ChooceSize = 35;						//������ʱ��Բѡ��Ĵ�С
const int Step = 2;								//����ʱ��ÿ����С
const int Eps = 5;								//����ʱ�ľ���������ʧ
const int Method = cv::NormTypes::NORM_L1;		//����ʱ�ľ�������
void GetCircularCenter(InputArray input,vector<Vec2f>& output)
{
    vector<Vec3f> vec;
    Mat img = input.getMat();
    Mat gray;

    
    cvtColor(img, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9),0,0);
    medianBlur(gray, gray, 5);

    HoughCircles(gray, vec, HOUGH_GRADIENT, 4, CircularDistance,100, 100, CircularMin, CircularMax);
    for (auto i : vec)
    {
        output.push_back({ i[0],i[1] });
    }
}
int GetEveryLine(InputArray pic, vector<Vec2f>& circularPos, vector <vector<int>>& everyIndex,vector<vector<cv::Ptr<bool>>>& recordRoad)
{
    //��ÿһ��Բ���б���
    int LineCount = 0;
    Mat image = pic.getMat();
    vector<int> s;

    everyIndex.resize(circularPos.size());
    recordRoad.resize(circularPos.size());
    for (int cirI = 1; cirI < circularPos.size(); cirI++)
    {
        
        for (int cirJ = 0; cirJ < cirI; cirJ++)
        {
            bool IsTrue = true;
            Vec2f nowPos = circularPos[cirI];
            Vec2f direction = circularPos[cirJ] - circularPos[cirI];
            direction=normalize(direction);

            nowPos += ChooceSize * direction;
            Vec2f targetPos = circularPos[cirJ] - ChooceSize * direction;
#ifdef DEBUG
            Mat testI;
            cvtColor(image, testI, COLOR_GRAY2BGR);

            for (auto i : circularPos)
            {
                circle(testI, Point(roundf(i[0]),roundf(i[1])), 3, Scalar(255, 0, 0), -1, 8, 0);
            }
#endif
            while (norm(nowPos, targetPos, Method) > Eps)
            {
                nowPos += Step * direction;
#ifdef DEBUG
                
                //line(testI, Point(roundf(circularPos[cirI][0]), roundf(circularPos[cirI][1])), Point(roundf(nowPos[0]), roundf(nowPos[1])), { 0,255,0 }, 5);
                //imshow("pic", image);
               // imshow("test", testI);
                //waitKey(10);
#endif
                if (image.at<uchar>(roundf(nowPos[1]), roundf(nowPos[0])) !=0)
                {
                    IsTrue = false;
                    break;
                }
            }
            if (IsTrue)
            {
                everyIndex[cirI].push_back(cirJ);
                everyIndex[cirJ].push_back(cirI);
                cv::Ptr<bool> type(new bool(false));
                recordRoad[cirI].push_back(type);
                recordRoad[cirJ].push_back(type);
                LineCount += 1;
            }
            
        }
    }
    return LineCount;
}
void GetBetterImage(InputArray input, OutputArray output)
{
    Mat img = input.getMat();
    
    cvtColor(img, img, COLOR_BGR2GRAY);
    medianBlur(img, img, 5);
    threshold(img, img, 145, 255, cv::ThresholdTypes::THRESH_BINARY);

    output.create(img.size(), img.type());
    img.copyTo(output);
    
}

bool GetResult(vector<vector<int>>&everyIndex,int lineCount, vector<vector<cv::Ptr<bool>>>& road, std::stack<pair<int, int>>& result)
{
    int nowRoadIndex=0;
    int nextRoadIndex = 0;
    int startIndex = 0;
    std::stack<pair<int, int>> pos;
    while (pos.size() < lineCount)
    {
        if (pos.empty())
        {
            nowRoadIndex = startIndex;
            startIndex++;
            nextRoadIndex = 0;
            if (startIndex == everyIndex.size())
            {
                return false;   //�޽�
            }
        }
        if (nextRoadIndex == everyIndex[nowRoadIndex].size())
        {
            nextRoadIndex = nowRoadIndex;
            auto just = pos.top();
            nowRoadIndex = just.first;
            nextRoadIndex = just.second;
            pos.pop();
            *road[nowRoadIndex][nextRoadIndex] = false;
            nextRoadIndex++;
            continue;
        }
        if (*road[nowRoadIndex][nextRoadIndex] == false)
        {
            *road[nowRoadIndex][nextRoadIndex] = true;
            pos.push({ nowRoadIndex,nextRoadIndex });
            nowRoadIndex = everyIndex[nowRoadIndex][nextRoadIndex];
            nextRoadIndex = 0;
        }
        else
        {
            nextRoadIndex++;
        }   
    }
    while (!pos.empty())
    {
        result.push(pos.top());
        pos.pop();
    }
    return true;
}

void FreeMemory(vector<vector<cv::Ptr<bool>>>& road)
{
    for (int i = 0; i < road.size(); i++)
    {
        for (int j = 0; j < road[i].size(); j++)
        {
            road[i][j].release();
        }
    }

}
int main(int argc, char** argv)
{

    Mat img = imread("3.jpg", IMREAD_COLOR);    //���ص�ͼƬ
    Mat testImg = img.clone();
    vector<Vec2f> info;
    vector<vector<cv::Ptr<bool>>> eve2;
    vector<vector<int>> eve;
    std::stack<pair<int, int>> pos;
    
    GetCircularCenter(img, info);   //��Բ(ʡ�鷳ֱ����API)

    GetBetterImage(img, img);       //����ͼƬ
    
    int lineCount=GetEveryLine(img, info, eve,eve2);    //����(�����������)
    
    
    GetResult(eve, lineCount, eve2,pos);                //�õ����(�����ٶ�ͦ����������ʵͦ���)
    int index = 50;
    imshow("test", testImg);                            //��ʾͼƬ


    //������
    while (!pos.empty())
    {
        int i= pos.top().first;
        int j=eve[pos.top().first][pos.top().second];
        std::cout << i << "\t" << j << "\n";
        line(testImg, Point(roundf(info[i][0]), roundf(info[i][1])), Point(roundf(info[j][0]), roundf(info[j][1])),Scalar(0,0,index), 5);
        pos.pop();
        index += 5;
        imshow("test",testImg);
        waitKey(500);       //ÿ500ms����һ����һ��

    }
    
    //���Ƴ�������
    for (int i = 0; i < eve.size(); i++)
    {
        
        for (int j = 0; j < eve[i].size(); j++)
        {
            
            line(testImg, Point(roundf(info[i][0]), roundf(info[i][1])), Point(roundf(info[eve[i][j]][0]), roundf(info[eve[i][j]][1])), Scalar(0, 255, 0), 5);
        }
        
    }
    FreeMemory(eve2);   //�����ڴ�
    imshow("test", testImg);
    waitKey(-1);
    return 0;
    
}