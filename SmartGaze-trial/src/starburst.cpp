// The SmartGaze Eye Tracker
// Copyright (C) 2016  Tristan Hume
// Released under GPLv2, see LICENSE file for full text

#include "starburst.h"
#include "ellipse.h"

#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iostream>

static const int kNumFilterSegments = 30;

using namespace cv;
using namespace std;

extern vector <Point2f> edge_point;
extern double pupil_param[5];
void starburst_pupil_contour_detection(Mat &m, Mat &validMask, Point2f start_point, int edge_thresh, int N, int minimum_candidate_features);
int* pupil_fitting_inliers(int width, int height, int &return_max_inliers);
#ifndef PI
#define PI 3.141592653589
#endif
static int LineCount=0;
static int picCount=0;
static int imgCount=0;
static char filename[255];
int starThresh = 16;
int starRays = 45;
RotatedRect findEllipseStarburst(Mat &m, const std::string &debugName) {
  // Gradient
  // Mat grad_x, grad_y, grad;
  // Mat abs_grad_x, abs_grad_y;
  // Scharr( m, grad_x, CV_16S, 1, 0);
  // convertScaleAbs( grad_x, abs_grad_x );
  // Scharr( m, grad_y, CV_16S, 0, 1);
  // convertScaleAbs( grad_y, abs_grad_y );
  // addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  // Blackest area
  Mat approxCenter;
  blur(m, approxCenter, Size(15,15));
  Point minLoc;
  double minVal;
  minMaxLoc(approxCenter, &minVal, nullptr, &minLoc, nullptr);
  threshold(approxCenter, approxCenter, minVal*1.5, 255, THRESH_BINARY);

  starburst_pupil_contour_detection(m, approxCenter, minLoc, starThresh, starRays, 1);
  std::sort(edge_point.begin(), edge_point.end(), [](Point2f a, Point2f b) {
      return b.y < a.y;
  });
  edge_point.resize((int)(edge_point.size()*(3.0/5.0)));

  Mat polarDebug = Mat::zeros(200,200, CV_8UC3);
  vector<pair<Point2f,Point2f>> polarPoints;
  for(Point2f p : edge_point) {
    Point2f offset = p - Point2f(minLoc.x, minLoc.y);
    Point2f polar;
    polar.x = atan2(offset.y, offset.x)*(180.0/PI);
    polar.y = sqrt(offset.x*offset.x + offset.y*offset.y);
    // circle(polarDebug, Point(polar.x,polar.y), 1, Scalar(0,255,0));
    polarPoints.push_back(pair<Point2f,Point2f>(polar, p));
  }
  std::sort(polarPoints.begin(), polarPoints.end(), [](pair<Point2f,Point2f> a, pair<Point2f,Point2f> b) {
      return a.first.x < b.first.x;
  });
  vector<Point2f> goodPoints;
  float segmentSize = polarPoints.size() / (float)(kNumFilterSegments);
  for(unsigned i = 0; i < kNumFilterSegments; i++) {
    auto start = polarPoints.begin()+std::min((size_t)std::round(i*segmentSize),polarPoints.size());
    auto stop = polarPoints.begin()+std::min((size_t)std::round((i+1)*segmentSize),polarPoints.size());
    if(stop-start == 0) break;
    std::sort(start, stop, [](pair<Point2f,Point2f> a, pair<Point2f,Point2f> b) {
        return a.first.y < b.first.y;
    });
    auto median = start+(stop-start)/2;
    goodPoints.push_back((*median).second);
    for(; start != stop; start++) {
      circle(polarDebug, Point(start->first.x,start->first.y), 1, Scalar(100,0,100 + (i*50 % 155)));
    }
    circle(polarDebug, Point(median->first.x,median->first.y), 2, Scalar(0,255,0));
  }
  imshow(debugName+"_polar", polarDebug);
  /*
  if (picCount<100){
    picCount++;
    cout<<"Polar Num:"<<picCount<<endl;
    sprintf(filename,"Polar%d.jpg",picCount);
    cout<<"Filename:"<<filename<<endl;
    imwrite(filename,polarDebug);
  }
  */


  int max_inliers_count;
  pupil_fitting_inliers(m.cols, m.rows, max_inliers_count);
  RotatedRect fittedIris2(Point2f(pupil_param[2],pupil_param[3]), Size2f(pupil_param[0]*2,pupil_param[1]*2), -pupil_param[4]*180/PI);
  RotatedRect fittedIris = (edge_point.size() < 5) ? RotatedRect() : fitEllipse(edge_point);
  RotatedRect fittedIris3 = (goodPoints.size() < 5) ? RotatedRect() : fitEllipse(goodPoints);

  ellipseScore(m, fittedIris3);

  Mat debugImage;
  cvtColor(m, debugImage, CV_GRAY2RGB);
  // for(Point2f p : edge_point) {
  //   Point intPt(p.x, p.y);
  //   circle(debugImage, intPt, 1, Scalar(0,255,0));
  // }
  for(Point2f p : goodPoints) {
    Point intPt(p.x, p.y);
    circle(debugImage, intPt, 2, Scalar(0,0,255));
  }
  circle(debugImage, minLoc, 2, Scalar(0,0,255));//blue cicle
  // ellipse(debugImage, fittedIris2, Scalar(255, 0, 200));
  ellipse(debugImage, fittedIris, Scalar(255, 255, 0));//yellow cicle
  ellipse(debugImage, fittedIris3, Scalar(0, 255, 255));
  circle(debugImage, fittedIris3.center, 2, Scalar(0,255, 0));//green points
  LineCount++;
  //cout<<"Line:"<<LineCount<<" Iris3 location"<<endl<<fittedIris3.center<<","<<debugName<<endl;
  cout<<fittedIris3.center<<"i"<<endl;
  imshow(debugName, debugImage);
/* 
  if(imgCount<100){
    imgCount++;
    //char  imgName=imgCount;//
    //cout<<"Iris Num:"<<imgCount<<endl;
    sprintf(filename,"Iris%d.jpg",imgCount);
    //cout<<"Filename:"<<filename<<endl;
    imwrite(filename,debugImage);

    
  }
*/  
  return RotatedRect();
}

// This project contains code from cvEyeTracker, another GPLv2 project, all code past this point in the file is modified from
// that project's code, included here under its GPLv2 license found in the LICENSE file, which is the same license SmartGaze uses.
// BEGIN CODE MODIFIED FROM cvEyeTracker
// cvEyeTracker - Version 1.2.5
// Part of the openEyes ToolKit -- http://hcvl.hci.iastate.edu/openEyes
// Authors : Dongheng Li <dhli@iastate.edu>
//           Derrick Parkhurst <derrick.parkhurst@hcvl.hci.iastate.edu>
//           Jason Babcock <babcock@nyu.edu>
//           David Winfield <dwinfiel@iastate.edu>
// Copyright (c) 2004-2006
// All Rights Reserved.


#include <vector>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "svd.h"


void get_5_random_num(int max_num, int* rand_num);
bool solve_ellipse(double* conic_param, double* pupil_param);
Point2f* normalize_edge_point(double &dis_scale, Point2f &nor_center, int ep_num);
void denormalize_ellipse_param(double* par, double* normalized_par, double dis_scale, Point2f nor_center);


void locate_edge_points(Mat &m, Mat &validMask, double cx, double cy, int dis, double angle_step, double angle_normal, double angle_spread, int edge_thresh);
Point2f get_edge_mean();

Point2f* normalize_point_set(Point2f* point_set, double &dis_scale, Point2f &nor_center, int num);

int inliers_num;
int angle_step = 20;    //20 degrees
double pupil_param[5] = {0, 0, 0, 0, 0};
vector <Point2f> edge_point;
vector <int> edge_intensity_diff;


//------------ Starburst pupil edge detection -----------//

// Input
// m: input image
// start_point: central start point of the feature detection process
// edge_thresh: best guess for the pupil contour threshold
// N: number of rays
// minimum_candidate_features: must return this many features or error
void starburst_pupil_contour_detection(Mat &m, Mat &validMask, Point2f start_point, int edge_thresh, int N, int minimum_candidate_features) {
  int dis = 7;
  double angle_spread = 100*PI/180;
  int loop_count = 0;
  double angle_step = 2*PI/N;
  double new_angle_step;
  Point2f edge, edge_mean;
  double angle_normal;
  double cx = start_point.x;
  double cy = start_point.y;
  int first_ep_num;

  while (edge_thresh > 5 && loop_count <= 10) {
    edge_intensity_diff.clear();
    edge_point.clear();
    while (edge_point.size() < minimum_candidate_features && edge_thresh > 5) {
      edge_intensity_diff.clear();
      edge_point.clear();
      locate_edge_points(m, validMask, cx, cy, dis, angle_step, 0, 2*PI, edge_thresh);
      if (edge_point.size() < minimum_candidate_features) {
        edge_thresh -= 1;
      }
    }
    if (edge_thresh <= 5) {
      break;
    }

    first_ep_num = edge_point.size();
    for (int i = 0; i < first_ep_num; i++) {
      edge = edge_point.at(i);
      angle_normal = atan2(cy-edge.y, cx-edge.x);
      new_angle_step = angle_step*(edge_thresh*1.0/edge_intensity_diff.at(i));
      locate_edge_points(m, validMask, edge.x, edge.y, dis, new_angle_step, angle_normal,
angle_spread, edge_thresh);
    }

    loop_count += 1;
    edge_mean = get_edge_mean();
    if (fabs(edge_mean.x-cx) + fabs(edge_mean.y-cy) < 10)
      break;

    cx = edge_mean.x;
    cy = edge_mean.y;
  }

  if (loop_count > 10) {
    edge_point.clear();
    printf("Error! edge points did not converge in %d iterations!\n", loop_count);
    return;
  }

  if (edge_thresh <= 5) {
    edge_point.clear();
    printf("Error! Adaptive threshold is too low!\n");
    return;
  }
}

void locate_edge_points(Mat &m, Mat &validMask, double cx, double cy, int dis, double angle_step, double angle_normal, double angle_spread, int edge_thresh) {
  double angle;
  Point2f p, edge;
  double dis_cos, dis_sin;
  uint8_t pixel_value1, pixel_value2;

  for (angle = angle_normal-angle_spread/2+0.0001; angle < angle_normal+angle_spread/2; angle += angle_step) {
    dis_cos = dis * cos(angle);
    dis_sin = dis * sin(angle);
    p.x = cx + dis_cos;
    p.y = cy + dis_sin;

    if (p.x < 0 || p.x >= m.cols || p.y < 0 || p.y >= m.rows)
      continue;
    pixel_value1 = m.at<uint8_t>((int)(p.y), (int)(p.x));
    while (1) {
      p.x += dis_cos;
      p.y += dis_sin;
      if (p.x < 0 || p.x >= m.cols || p.y < 0 || p.y >= m.rows)
        break;

      pixel_value2 = m.at<uint8_t>((int)(p.y), (int)(p.x));
      bool is_valid = validMask.at<uint8_t>((int)(p.y - dis_sin/2), (int)(p.x - dis_cos/2)) > 0;
      if (pixel_value2 - pixel_value1 > edge_thresh && is_valid) {
        edge.x = p.x - dis_cos/2;
        edge.y = p.y - dis_sin/2;
        edge_point.push_back(edge);
        edge_intensity_diff.push_back(pixel_value2 - pixel_value1);
        break;
      }
      pixel_value1 = pixel_value2;
    }
  }
}

Point2f get_edge_mean() {
  Point2f edge;
  int i;
  double sumx=0, sumy=0;
  Point2f edge_mean;
  for (i = 0; i < edge_point.size(); i++) {
    edge = edge_point.at(i);
    sumx += edge.x;
    sumy += edge.y;
  }
  if (edge_point.size() != 0) {
    edge_mean.x = sumx / edge_point.size();
    edge_mean.y = sumy / edge_point.size();
  } else {
    edge_mean.x = -1;
    edge_mean.y = -1;
  }
  return edge_mean;
}

//------------ Ransac ellipse fitting -----------//
// Randomly select 5 indeics
void get_5_random_num(int max_num, int* rand_num) {
  int rand_index = 0;
  int r;
  int i;
  bool is_new = 1;

  if (max_num == 4) {
    for (i = 0; i < 5; i++) {
      rand_num[i] = i;
    }
    return;
  }

  while (rand_index < 5) {
    is_new = 1;
    r = (int)((rand()*1.0/RAND_MAX) * max_num);
    for (i = 0; i < rand_index; i++) {
      if (r == rand_num[i]) {
        is_new = 0;
        break;
      }
    }
    if (is_new) {
      rand_num[rand_index] = r;
      rand_index++;
    }
  }
}


// solve_ellipse
// conic_param[6] is the parameters of a conic {a, b, c, d, e, f}; conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
// ellipse_param[5] is the parameters of an ellipse {ellipse_a, ellipse_b, cx, cy, theta}; a & b is the major or minor axis;
// cx & cy is the ellipse center; theta is the ellipse orientation
bool solve_ellipse(double* conic_param, double* ellipse_param) {
  double a = conic_param[0];
  double b = conic_param[1];
  double c = conic_param[2];
  double d = conic_param[3];
  double e = conic_param[4];
  double f = conic_param[5];
  //get ellipse orientation
  double theta = atan2(b, a-c)/2;

  //get scaled major/minor axes
  double ct = cos(theta);
  double st = sin(theta);
  double ap = a*ct*ct + b*ct*st + c*st*st;
  double cp = a*st*st - b*ct*st + c*ct*ct;

  //get translations
  double cx = (2*c*d - b*e) / (b*b - 4*a*c);
  double cy = (2*a*e - b*d) / (b*b - 4*a*c);

  //get scale factor
  double val = a*cx*cx + b*cx*cy + c*cy*cy;
  double scale_inv = val - f;

  if (scale_inv/ap <= 0 || scale_inv/cp <= 0) {
    //printf("Error! ellipse parameters are imaginary a=sqrt(%lf), b=sqrt(%lf)\n", scale_inv/ap, scale_inv/cp);
    memset(ellipse_param, 0, sizeof(double)*5);
    return 0;
  }

  ellipse_param[0] = sqrt(scale_inv / ap);
  ellipse_param[1] = sqrt(scale_inv / cp);
  ellipse_param[2] = cx;
  ellipse_param[3] = cy;
  ellipse_param[4] = theta;
  return 1;
}

Point2f* normalize_edge_point(double &dis_scale, Point2f &nor_center, int ep_num) {
  double sumx = 0, sumy = 0;
  double sumdis = 0;
  Point2f edge;
  int i;
  for (i = 0; i < ep_num; i++) {
    edge = edge_point.at(i);
    sumx += edge.x;
    sumy += edge.y;
    sumdis += sqrt((double)(edge.x*edge.x + edge.y*edge.y));
  }

  dis_scale = sqrt((double)2)*ep_num/sumdis;
  nor_center.x = sumx*1.0/ep_num;
  nor_center.y = sumy*1.0/ep_num;
  Point2f *edge_point_nor = (Point2f*)malloc(sizeof(Point2f)*ep_num);
  for (i = 0; i < ep_num; i++) {
    edge = edge_point.at(i);
    edge_point_nor[i].x = (edge.x - nor_center.x)*dis_scale;
    edge_point_nor[i].y = (edge.y - nor_center.y)*dis_scale;
  }
  return edge_point_nor;
}

void denormalize_ellipse_param(double* par, double* normalized_par, double dis_scale, Point2f nor_center) {
    par[0] = normalized_par[0] / dis_scale;  //major or minor axis
    par[1] = normalized_par[1] / dis_scale;
    par[2] = normalized_par[2] / dis_scale + nor_center.x; //ellipse center
    par[3] = normalized_par[3] / dis_scale + nor_center.y;
}

int* pupil_fitting_inliers(int width, int height,  int &return_max_inliers_num) {
  int i;
  int ep_num = edge_point.size();   //ep stands for edge point
  Point2f nor_center;
  double dis_scale;

  int ellipse_point_num = 5;  //number of point that needed to fit an ellipse
  if (ep_num < ellipse_point_num) {
    printf("Error! %d points are not enough to fit ellipse\n", ep_num);
    memset(pupil_param, 0, sizeof(pupil_param));
    return_max_inliers_num = 0;
    return NULL;
  }

  //Normalization
  Point2f *edge_point_nor = normalize_edge_point(dis_scale, nor_center, ep_num);

  //Ransac
  int *inliers_index = (int*)malloc(sizeof(int)*ep_num);
  int *max_inliers_index = (int*)malloc(sizeof(int)*ep_num);
  int ninliers = 0;
  int max_inliers = 0;
  int sample_num = 1000;  //number of sample
  int ransac_count = 0;
  double dis_threshold = sqrt(3.84)*dis_scale;
  double dis_error;

  memset(inliers_index, int(0), sizeof(int)*ep_num);
  memset(max_inliers_index, int(0), sizeof(int)*ep_num);
  int rand_index[5];
  double A[6][6];
  int M = 6, N = 6; //M is row; N is column
  for (i = 0; i < N; i++) {
    A[i][5] = 1;
    A[5][i] = 0;
  }
  double **ppa = (double**)malloc(sizeof(double*)*M);
  double **ppu = (double**)malloc(sizeof(double*)*M);
  double **ppv = (double**)malloc(sizeof(double*)*N);
  for (i = 0; i < M; i++) {
    ppa[i] = A[i];
    ppu[i] = (double*)malloc(sizeof(double)*N);
  }
  for (i = 0; i < N; i++) {
    ppv[i] = (double*)malloc(sizeof(double)*N);
  }
  double pd[6];
  int min_d_index;
  double conic_par[6] = {0};
  double ellipse_par[5] = {0};
  double best_ellipse_par[5] = {0};
  double ratio;
  while (sample_num > ransac_count) {
    get_5_random_num((ep_num-1), rand_index);

    //svd decomposition to solve the ellipse parameter
    for (i = 0; i < 5; i++) {
      A[i][0] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].x;
      A[i][1] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].y;
      A[i][2] = edge_point_nor[rand_index[i]].y * edge_point_nor[rand_index[i]].y;
      A[i][3] = edge_point_nor[rand_index[i]].x;
      A[i][4] = edge_point_nor[rand_index[i]].y;
    }

    svd(M, N, ppa, ppu, pd, ppv);
    min_d_index = 0;
    for (i = 1; i < N; i++) {
      if (pd[i] < pd[min_d_index])
        min_d_index = i;
    }

    for (i = 0; i < N; i++)
      conic_par[i] = ppv[i][min_d_index]; //the column of v that corresponds to the smallest singular value,
                                                //which is the solution of the equations
    ninliers = 0;
    memset(inliers_index, 0, sizeof(int)*ep_num);
    for (i = 0; i < ep_num; i++) {
      dis_error = conic_par[0]*edge_point_nor[i].x*edge_point_nor[i].x +
                  conic_par[1]*edge_point_nor[i].x*edge_point_nor[i].y +
                  conic_par[2]*edge_point_nor[i].y*edge_point_nor[i].y +
                  conic_par[3]*edge_point_nor[i].x + conic_par[4]*edge_point_nor[i].y + conic_par[5];
      if (fabs(dis_error) < dis_threshold) {
        inliers_index[ninliers] = i;
        ninliers++;
      }
    }

    if (ninliers > max_inliers) {
      if (solve_ellipse(conic_par, ellipse_par)) {
        denormalize_ellipse_param(ellipse_par, ellipse_par, dis_scale, nor_center);
        ratio = ellipse_par[0] / ellipse_par[1];
        if (ellipse_par[2] > 0 && ellipse_par[2] <= width-1 && ellipse_par[3] > 0 && ellipse_par[3] <= height-1 &&
            ratio > 0.5 && ratio < 2) {
          memcpy(max_inliers_index, inliers_index, sizeof(int)*ep_num);
          for (i = 0; i < 5; i++) {
            best_ellipse_par[i] = ellipse_par[i];
          }
          max_inliers = ninliers;
          sample_num = (int)(log((double)(1-0.99))/log(1.0-pow(ninliers*1.0/ep_num, 5)));
        }
      }
    }
    ransac_count++;
    if (ransac_count > 1500) {
      printf("Error! ransac_count exceed! ransac break! sample_num=%d, ransac_count=%d\n", sample_num, ransac_count);
      break;
    }
  }
  //INFO("ransc end\n");
  if (best_ellipse_par[0] > 0 && best_ellipse_par[1] > 0) {
    for (i = 0; i < 5; i++) {
      pupil_param[i] = best_ellipse_par[i];
    }
  } else {
    memset(pupil_param, 0, sizeof(pupil_param));
    max_inliers = 0;
    free(max_inliers_index);
    max_inliers_index = NULL;
  }

  for (i = 0; i < M; i++) {
    free(ppu[i]);
    free(ppv[i]);
  }
  free(ppu);
  free(ppv);
  free(ppa);

  free(edge_point_nor);
  free(inliers_index);
  return_max_inliers_num = max_inliers;
  return max_inliers_index;
}
