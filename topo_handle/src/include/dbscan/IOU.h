#pragma once
#ifndef _IOU_H_
#define _IOU_H_
#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include <algorithm>
#include <utility>
#include <math.h>
struct Point2f
{
	float x,y;
};
//构建矩形时顶点顺序可以为顺时针也可以为逆时针
class proposalType{
public:
	float x1=0, x2=0, x3=0, x4=0;
	float y1=0, y2=0, y3=0, y4=0;
	float score = 0;
	proposalType(std::vector<std::vector<int>> inputs){
		x1 = inputs[0][0];	y1 = inputs[0][1];
		x2 = inputs[1][0];	y2 = inputs[1][1];
		x3 = inputs[2][0];	y3 = inputs[2][1];
		x4 = inputs[3][0];	y4 = inputs[3][1];
	}
	proposalType(std::vector<Point2f>inputs){
		x1 = inputs[0].x; y1 = inputs[0].y;
		x2 = inputs[1].x; y2 = inputs[1].y;
		x3 = inputs[2].x; y3 = inputs[2].y;
		x4 = inputs[3].x; y4 = inputs[3].y;
	}
};
float IOU(const proposalType& Rect1, const proposalType& Rect2);

const float eps = 1e-8;
int fcmp(float x){
	if(x>eps)	return 1;
	return x<-eps?-1:0;
}
//求叉积
float cross(Point2f a, Point2f b, Point2f c){
	return (a.x-c.x)*(b.y-c.y)-(b.x-c.x)*(a.y-c.y);
}
// void swap(Point2f a, Point2f b){
// 	Point2f tmp = std::move(a);
// 	a = std::move(b);
// 	b = std::move(tmp);
// }
//求两个矩形交点
Point2f intersection(Point2f a,Point2f b,Point2f c,Point2f d){
    Point2f p = a;
    float t =((a.x-c.x)*(c.y-d.y)-(a.y-c.y)*(c.x-d.x))/((a.x-b.x)*(c.y-d.y)-(a.y-b.y)*(c.x-d.x));
    p.x +=(b.x-a.x)*t;
    p.y +=(b.y-a.y)*t;
    return p;
}
//计算多边形面积
float PolygonArea(Point2f p[], int n){
    if(n < 3) return 0.0;
    float s = p[0].y * (p[n - 1].x - p[1].x);
    p[n] = p[0];
    for(int i = 1; i < n; ++ i)
        s += p[i].y * (p[i - 1].x - p[i + 1].x);
    return fabs(s * 0.5);
}
//ConvexPolygonIntersectArea凸多边形相交区域
float CPIA(Point2f a[], Point2f b[], int na, int nb){
    Point2f p[20], tmp[20];
    int tn, sflag, eflag;
    a[na] = a[0], b[nb] = b[0];
    memcpy(p,b,sizeof(Point2f)*(nb + 1));
    for(int i = 0; i < na && nb > 2; i++)
    {
        sflag = fcmp(cross(a[i + 1], p[0],a[i]));
        for(int j = tn = 0; j < nb; j++, sflag = eflag)
        {
            if(sflag>=0) tmp[tn++] = p[j];
            eflag = fcmp(cross(a[i + 1], p[j + 1],a[i]));
            if((sflag ^ eflag) == -2)
                tmp[tn++] = intersection(a[i], a[i + 1], p[j], p[j + 1]); ///求交点
        }
        memcpy(p, tmp, sizeof(Point2f) * tn);
        nb = tn, p[nb] = p[0];
    }
    if(nb < 3) return 0.0;
    return PolygonArea(p, nb);
}
//SimplePolygonIntersectArea 简单多边形相交区域
float SPIA(Point2f a[], Point2f b[], int na, int nb){
    int i, j;
    Point2f t1[4], t2[4];
    float res = 0, num1, num2;
    a[na] = t1[0] = a[0], b[nb] = t2[0] = b[0];
 
    for(i = 2; i < na; i++)
    {
        t1[1] = a[i-1], t1[2] = a[i];
        num1 = fcmp(cross(t1[1], t1[2],t1[0]));
        if(num1 < 0) std::swap(t1[1], t1[2]);
 
        for(j = 2; j < nb; j++)
        {
 
            t2[1] = b[j - 1], t2[2] = b[j];
            num2 = fcmp(cross(t2[1], t2[2],t2[0]));
            if(num2 < 0) std::swap(t2[1], t2[2]);
            res += CPIA(t1, t2, 3, 3) * num1 * num2;
        }
    }
    return res;
}
//求交集面积
float intersection_area(const proposalType & r1, const proposalType & r2){
    Point2f p1[10],p2[10];
    
    p1[0].x=r1.x2;
    p1[0].y=r1.y2;
    p1[1].x=r1.x3;
    p1[1].y=r1.y3;
    p1[2].x=r1.x4;
    p1[2].y=r1.y4;
    p1[3].x=r1.x1;
    p1[3].y=r1.y1;
 
    p2[0].x=r2.x2;
    p2[0].y=r2.y2;
    p2[1].x=r2.x3;
    p2[1].y=r2.y3;
    p2[2].x=r2.x4;
    p2[2].y=r2.y4;
    p2[3].x=r2.x1;
    p2[3].y=r2.y1;
    float area = SPIA(p1, p2, 4, 4);
    return area;
}
float calcularea(const proposalType & r){
    float d12=sqrt(pow(r.x2-r.x1,2)+pow(r.y2-r.y1,2));
    float d14=sqrt(pow(r.x4-r.x1,2)+pow(r.y4-r.y1,2));
    float d24=sqrt(pow(r.x2-r.x4,2)+pow(r.y2-r.y4,2));
    float d32=sqrt(pow(r.x2-r.x3,2)+pow(r.y2-r.y3,2));
    float d34=sqrt(pow(r.x3-r.x4,2)+pow(r.y3-r.y4,2));
    float p1=(d12+d14+d24)/2;
    float p2=(d24+d32+d34)/2;
    float s1=sqrt(p1*(p1-d12)*(p1-d14)*(p1-d24));
    float s2=sqrt(p2*(p2-d32)*(p2-d34)*(p2-d24));
    return s1+s2;
}
bool cmpr(const proposalType &a,const proposalType &b){
    return a.score > b.score;
}
// void nms(std::vector<proposalType>& proposals, const float nms_threshold){
 
//     //按分数排序
//     sort(proposals.begin(),proposals.end(),cmpr);
//     //标志，false代表留下，true代表扔掉
//     std::vector<bool> del(proposals.size(), false);
//     for(size_t i = 0; i < proposals.size(); i++){
//         if(!del[i]){
//             for(size_t j = i+1; j < proposals.size()-1; j++){
//                 if(!del[j] && IOU(proposals[i], proposals[j]) > nms_threshold){
//                     del[j] = true;//IOU大于阈值，扔掉
//                 }
//             }
//         }
//     }

//     std::vector<proposalType> new_proposals;

//     for(int i=0;i<proposals.size();i++){
//         if(!del[i]) new_proposals.push_back(proposals[i]);
//     }
//     proposals.clear();
//     std::vector<proposalType>().std::swap(proposals);
//     proposals = new_proposals;
// }
//求交并比的两个矩形顶点排列顺序必须相同，都为顺时针或者都为逆时针
float IOU(const proposalType & r1, const proposalType & r2, const int& state){    
    float inter = intersection_area(r1,r2);
 	// cout<<"inter   "<<inter<<endl;
 	// cout<<"        "<<calcularea(r1)<<" "<<calcularea(r2)<<endl;
    // float o = inter / (calcularea(r1) + calcularea(r2) - inter);//交并比
    if(state==0){
    	float o = inter/calcularea(r1);
    	return (o >= 0) ? o : 0;
	}
	if(state == 1){
		float o = inter/calcularea(r2);
    	return (o >= 0) ? o : 0;
	}
	if(state == 2){
		double area1 = calcularea(r1);	double area2 = calcularea(r2);
		if(fabs(inter - area1)<1e-2 || fabs(inter - area2)<1e-2){
			return 1.0;
		}
		float o = inter / (area1 + area2 - inter);
		return (o >= 0) ? o : 0;
	}
	return 0;
}
#endif