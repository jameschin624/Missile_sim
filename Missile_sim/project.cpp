#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <new>
#include <cmath>
#include <graphics.h>
#include <time.h>
//#include <Windows.h>
using namespace std;
#define OutOfView 10000
#define SIZE 4 //scaling factor
const float Pi=3.1415926;
const float cAcc=0.3; //coefficient for the degree of acceleration function
const float lapse=0.05,tDelta=0.1; //data duration for missile data collecting
const int wx=1400, wy=700; //視窗大小 
const int x_offset=20, y_offset=20;
int wx1=wx+2*x_offset;
int wy1=wy+2*y_offset;
int tSpan=10/tDelta; //max number of times to intercept the missile
int nCollect=tSpan/2; //number of missile data collected
int OFFSET=nCollect-20; //number of missile data ignored from beginning
int tDue=7/tDelta; //火花存活最長計數 (tDue+30) 
float fx=3000,fy=3000,fz=300,lFocus=fx/3; //焦點及焦距 
int color;
int tCount=0;
int nSetup=30; //time needed to analyze data and set up for interceptor
float vMax=300; //initial value of velocity for interceptor
float aStart=Pi/6; //min elevated angle for interceptor
float dMin=20; //accepted distance while hitting between missile and interceptor
float dMax=500; //rejected distance while hitting between missile and interceptor

class point
{
	public:
	float x,y,z;
	point()
	{
		x=0;
		y=0;
		z=0;
		return;
	}
		
	point(float xx,float yy,float zz)
	{
		x=xx;
		y=yy;
		z=zz;
		return;
	}	
	
	float length()
	{
		return pow(x*x+y*y+z*z,0.5);
	}
};

class vector
{
	public:
	point target;
	float length;
	
	vector(point p2)
	{
		target.x=p2.x;
		target.y=p2.y;
		target.z=p2.z;
		length=pow(pow(p2.x,2)+pow(p2.y,2)+pow(p2.z,2),0.5);
		return;
	}	
	
	vector(point p1,point p2)
	{

		target.x=p2.x-p1.x;
		target.y=p2.y-p1.y;
		target.z=p2.z-p1.z;
		length=pow(pow(target.x,2)+pow(target.y,2)+pow(target.z,2),0.5);
		return;	
	}
	
	vector()
	{
		target.x=0;
		target.y=0;
		target.z=0;
		length=0;
		return;
	}	
	
	vector uv()
	{
		point p;
		p.x=(target.x)/length;
		p.y=(target.y)/length;
		p.z=(target.z)/length;
		return p;
	}
	
	vector translate(point p)
	{
		p.x+=target.x;
		p.y+=target.y;
		p.z+=target.z;
		return p;
	}
	
	vector scale(float s)
	{
		point p;
		p.x=(target.x)*s;
		p.y=(target.y)*s;
		p.z=(target.z)*s;
		return p;
	}	
	
	vector rotate(int axis,float angle)//axis=1 for x, 2 for y, 3 for z axis
	{
		point p;
		switch(axis)
		{
		case 1: 
			p.x=target.x;
			p.y=target.y*cos(angle)-target.z*sin(angle);
			p.z=target.y*sin(angle)+target.z*cos(angle);
			break;
		case 2: 
			p.y=target.y;
			p.x=target.x*cos(angle)+target.z*sin(angle);
			p.z=-target.x*sin(angle)+target.z*cos(angle);
			break;
		case 3: 
			p.z=target.z;
			p.x=target.x*cos(angle)-target.y*sin(angle);
			p.y=target.x*sin(angle)+target.y*cos(angle);
			break;		
		}
		return p;
	}
	
	
	float inProduct(point p)
	{
		return (target.x)*(p.x)+(target.y)*(p.y)+(target.z)*(p.z);
	}
	
	vector outProduct(point p)
	{
		point q;
		q.x=(target.y)*(p.z)-(target.z)*(p.y);
		q.y=(target.z)*(p.x)-(target.x)*(p.z);
		q.z=(target.x)*(p.y)-(target.y)*(p.x);
		return q;
	}
	
	float angle(point p)
	{
		float a=inProduct(p)/length/p.length();
		return acos(a);
	}
};


class statistics
{
	public:
	point *data;
	point *diff1;
	point *diff2;
	int nData;
	point abc[3]; //parameters for acceleration funcition, 0 for X,1 for Y, 2 for Z
	
	statistics(int n,int m) //n:number of data  m:which missile track file to be analyzed
	{
		FILE *fp;
		int i;
		char fn[30];
		sprintf(fn,"track%d.txt",m);
		fp=fopen(fn,"r");
		if (fp==NULL) return;
		nData=n;
		data=new point[n];
		for (i=0;i<n;i++)
		{
			fscanf(fp,"%f %f %f",&(data[i].x),&(data[i].y),&(data[i].z));
		}
		diff1=new point[n];	
		for (i=0;i<n-1;i++)
		{
			diff1[i].x=(data[i+1].x-data[i].x)/tDelta;
			diff1[i].y=(data[i+1].y-data[i].y)/tDelta;
			diff1[i].z=(data[i+1].z-data[i].z)/tDelta;
		}
		diff2=new point[n];
		for (i=0;i<n-2;i++)
		{
			diff2[i].x=(diff1[i+1].x-diff1[i].x)/tDelta;
			diff2[i].y=(diff1[i+1].y-diff1[i].y)/tDelta;
			diff2[i].z=(diff1[i+1].z-diff1[i].z)/tDelta;
		}
		fclose(fp); 
		return;
	}	
	
	statistics(int n, point *data1)
	{
		int i;
		nData=n;
		data=data1;
		diff1=new point[n];
		for (i=0;i<n-1;i++)
		{
			diff1[i].x=(data[i+1].x-data[i].x)/tDelta;
			diff1[i].y=(data[i+1].y-data[i].y)/tDelta;
			diff1[i].z=(data[i+1].z-data[i].z)/tDelta;
		}
		diff2=new point[n];
		for (i=0;i<n-2;i++)
		{
			diff2[i].x=(diff1[i+1].x-diff1[i].x)/tDelta;
			diff2[i].y=(diff1[i+1].y-diff1[i].y)/tDelta;
			diff2[i].z=(diff1[i+1].z-diff1[i].z)/tDelta;
		}
		return;
	}	
	
	point pSum() //summation of X or Y or Z
	{
		int i;
		point sum;
		sum.x=0;
		sum.y=0;
		sum.z=0;
		
		for (i=0;i<nData;i++)
		{
			sum.x+=data[i].x;
			sum.y+=data[i].y;
			sum.z+=data[i].z;
		}
		return sum;
	}
	
	point pSum2() //summation of X*X or Y*Y or Z*Z
	{
		int i;
		point sum;
		sum.x=0;
		sum.y=0;
		sum.z=0;
		
		for (i=0;i<nData;i++)
		{
			sum.x+=pow(data[i].x,2);
			sum.y+=pow(data[i].y,2);
			sum.z+=pow(data[i].z,2);
		}
		return sum;
	}
	
	float fSumXY() //summation of X*Y
	{
		int i;
		float sum=0;
		
		for (i=0;i<nData;i++)
		{
			sum+=data[i].x * data[i].y;
		}
		return sum;
	}
	
	point pAvg() //average of X or Y or Z
	{
		point avg,sum;
		sum=pSum();		
		avg.x=sum.x/nData;
		avg.y=sum.y/nData;
		avg.z=sum.z/nData;		
		return avg;
	}
	
	point mse() //average of X or Y or Z
	{
		point pT1,pT2,ab;
		float xy=fSumXY();
		pT1=pSum();
		pT2=pSum2();
		ab.x=(xy*nData-pT1.x*pT1.y)/(pT2.x*nData-pT1.x*pT1.x);
		ab.y=(pT2.x*pT1.y-pT1.x*xy)/(pT2.x*nData-pT1.x*pT1.x);		
		return ab;
	}
	
	point sd() //standard deviation of X or Y or Z
	{
		point pT1,pT2,ab;
		pT1=pAvg();
		pT2=pSum2();
		ab.x=pow((pT2.x/nData-pT1.x*pT1.x),0.5);
		ab.y=pow((pT2.y/nData-pT1.y*pT1.y),0.5);
		ab.z=pow((pT2.z/nData-pT1.z*pT1.z),0.5);	
		return ab;
	}
	
	void accFn() //抽樣 missile 的 nCollect 筆 加速度資料, 利用 MSE 求得加速度預測公式  
	{
		int i;
		int o=OFFSET; 
		nData=nData-o-2; 
		for (i=0;i<nData;i++)
		{
			data[i].x=1/pow((i+o)*tDelta+1,cAcc);
			data[i].z=0;
		}
		for (i=0;i<nData;i++)
			data[i].y=diff2[i+o].x;	
		abc[0]=mse();
		for (i=0;i<nData;i++)
			data[i].y=diff2[i+o].y;	
		abc[1]=mse();
		for (i=0;i<nData;i++)
			data[i].y=diff2[i+o].z;	
		abc[2]=mse();
		return;		
	}
};



class view
{
	public:
	float x,y;
	float xCenter,yCenter;
	point origin;//視窗之原點3D座標 
	point vx,vy;//視窗之 X 及 Y 軸單位向量 
	point fp; //focus point 之3D座標 
	float fl; //focus length
	
	view()
	{
		fp.x=fx;
		fp.y=fy;
		fp.z=fz;
		fl=lFocus;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp);
		origin=vTemp1.uv().scale(-fl).translate(fp).target;	
		point p0(0,0,-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.outProduct(vy).uv().target;
		return;
	}
	
	view(point p,point p1,float l)//
	{
		fp.x=p.x;
		fp.y=p.y;
		fp.z=p.z;
		fl=l;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp,p1);
		origin=vTemp1.uv().scale(fl).translate(fp).target;	
		point p0(p1.x,p1.y,p1.z-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.scale(-1).outProduct(vy).uv().target;
		return;
	}
	
	point pProj3D(point p) //p點投影後之3D座標 
	{
		vector vTemp0(fp);
		vector vTemp1(fp,p);
		point p0=vTemp1.uv().scale(fl/cos(vTemp1.uv().angle(vTemp0.scale(-1).uv().target))).translate(fp).target;
		return p0;//vTemp1.uv().scale(fl/cos(vTemp1.angle(vTemp0.scale(-1).target))).translate(fp).target;
	}
	
	void Proj2D(point p) // p點投影後之視窗座標 
	{
		vector vT1(origin,p),vT2(fp,origin);
		if (vT1.inProduct(vT2.target) <= 0)
		{
			x=OutOfView;
			y=OutOfView;
			return;
		}
		point p0=pProj3D(p);
		vector vTemp1(origin,p0);
		x=xCenter+vTemp1.inProduct(vx);
		y=yCenter+vTemp1.inProduct(vy);
		return;	
	}
	
	void d3Point(point p) //視窗中畫出 P點 
	{
		point p0(0,0,0);
		vector v1(fp,p),v2(fp,p0);
		Proj2D(p);
		if (x==OutOfView) return;
		//putpixel(x,y,color);
		circle(x,y,1/(v1.length*cos(v1.angle(v2.target)))*fp.x*2+1);
		return;
	}
	
	void d3LineEmit(point p,point d) //p:source point  d:direction vector 視窗中畫射線 
	{
		Proj2D(p);
		if (x==OutOfView) return;
		float xT1=x,yT1=y;
		while (x>=0 && x<wx1 && y>=0 && y<wy1)
		{
			p.x+=d.x*10;
			p.y+=d.y*10;
			p.z+=d.z*10;
			Proj2D(p);	
			
			if (x==OutOfView) return;
			putpixel(x,y,10);
		}
		return;
	}
		
	void d3LineSeg(point p1,point p2) //視窗中畫線段 
	{
		Proj2D(p1);
		if (x==OutOfView) return;
		float xT=x, yT=y;
		Proj2D(p2);
		if (x==OutOfView) return;
		float distance=pow(pow(x-xT,2)+pow(y-yT,2),0.5);
		if (x<(-distance) || xT<(-distance) || x>(wx1+distance) || xT>(wx1+distance)) return;
		if (y<(-distance) || yT<(-distance) || y>(wy1+distance) || yT>(wy1+distance)) return;
		line(x,y,xT,yT);
		return;
	}	
};

class missile
{
	public:
	point start,current;
	point v;
	point a;
	float g0,g1,g2;
	float r0,r1,r2;
	float n0;
	point *shape;
	int **neighbor;
	int nVertex;
	int id;
	int on;
	//time_t tStart,tCurrent;
	view w;
	
	missile()
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		start.x=0;	
		start.y=0;
		start.z=0;
		v.x=120;
		v.y=60;
		v.z=120;
		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-3;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	missile(char c)
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		
		start.x=(rand()%500);	
		start.y=(rand()%500);
		start.z=0;
		v.x=(rand()%120);
		v.y=(rand()%120);
		v.z=(rand()%120);
		g0=9.8;
		r0=(float) (rand()%100)/100;
		r1=(float) (rand()%100)/1000;
		r2=(float) (rand()%100)/10000;
		n0=(float) (rand()%10)/10-5;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	missile(point p0,point v0)
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		start=p0;	
		v=v0;
		g0=9.8;
		r0=(float) (rand()%100)/100;
		r1=(float) (rand()%100)/1000;
		r2=(float) (rand()%100)/10000;
		n0=(float) (rand()%10)/10-5;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	missile(int f)//missile parameters from file 
	{
		FILE *fp;
		char fn[30];
		int i,j,n,temp; 
		id=f;
		sprintf(fn,"missile%d.txt",f);
		fp=fopen(fn,"r");
		if (fp==NULL) return;
		fscanf(fp,"%d",&nVertex);
		shape=new point[nVertex];
		neighbor=new int*[nVertex];
		
		
		for(i=0;i<nVertex;i++)
		{
			fscanf(fp,"%f %f %f",&(shape[i].x),&(shape[i].y),&(shape[i].z));
			neighbor[i]=new int[nVertex];
			for (j=0;j<nVertex;j++)
				neighbor[i][j]=0;
		}
		for(i=0;i<nVertex-1;i++)
		{
			fscanf(fp,"%d",&n);
			for (j=0;j<n;j++)
			{
				fscanf(fp,"%d",&temp);
				neighbor[i][temp]=1;
				neighbor[temp][i]=1;
			}
		}
		fscanf(fp,"%f",&n0);	
		fscanf(fp,"%f",&r0);	
		fscanf(fp,"%f",&r1);
		fscanf(fp,"%f",&r2);	
		fscanf(fp,"%f",&g0);
		fscanf(fp,"%f",&g1);
		fscanf(fp,"%f",&g2);	
		fscanf(fp,"%f %f %f",&(start.x),&(start.y),&(start.z));
		fscanf(fp,"%f %f %f",&(v.x),&(v.y),&(v.z));
		cout<<n0<<","<<r0<<","<<r1<<","<<r2<<","<<g0<<","<<endl;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	vector pNext(float t) //計算 T時間後之位置 
	{
		vector vv(v);
		char tt[30];
		float r = (pow(vv.length,2)*r2 + vv.length*r1 + r0)/vv.length;
		a.x=r*(-v.x)+n0*v.y/vv.length;
		a.y=r*(-v.y)+n0*(-v.x)/vv.length;
		a.z=r*(-v.z)-g0;
		current.x+=v.x*t+0.5*a.x*t*t;
		current.y+=v.y*t+0.5*a.y*t*t;
		current.z+=v.z*t+0.5*a.z*t*t;
		//tCurrent+=t;
		v.x=v.x+a.x*t;
		v.y=v.y+a.y*t;
		v.z=v.z+a.z*t;
		setcolor(15);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",v.x,v.y,v.z);
		outtextxy(40,100*(id-1)+50,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",current.x,current.y,current.z);
		outtextxy(40,100*(id-1)+30,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",a.x,a.y,a.z);
		outtextxy(40,100*(id-1)+70,tt);	
		setcolor(color);
		//getch();
		return current;		
	}	
	
void show(int color) //於座標(X,Y,Z)畫出 MISSILE 
{
	if (on==0) return;
	setcolor(color);
	point p2=shape[0];
	if (p2.x==0 && p2.y==0 && p2.z==0)
	{
		w.d3Point(current);
		return;
	}
	vector *temp=new vector[nVertex];	
	vector v1(v),v2(p2);
	int i,j;
	float a1=v1.angle(p2);
	v2=v2.rotate(2,a1);
	p2.x=v2.target.x;
	p2.y=v2.target.y;
	p2.z=0;
	v1.target.z=0;
	float a2=v1.angle(p2);
	v1=v1.outProduct(p2);
	if (v1.target.z>0) 
		a2=-a2;
	for (i=0;i<nVertex;i++)
	{
		temp[i].target=shape[i];
		temp[i]=temp[i].rotate(2,a1).rotate(3,a2).scale(SIZE).translate(current); 
	}
	for (i=0;i<nVertex;i++)
		for (j=0;j<i;j++)
		{
			if (neighbor[i][j]==1)
				w.d3LineSeg(temp[i].target,temp[j].target);
		}         
	return;
}
};

class interceptor
{
	public:
	point start,current;
	point v;
	point a;
	float g0,g1,g2;
	float r0,r1,r2;
	float n0;
	point *shape;
	int **neighbor;
	int nVertex;
	int id;
	int on;
	//time_t tStart,tCurrent;
	view w;
	point xabc[3]; //predicted parameter of acceleration of missile after t1
	int tSetup;    // tSetup=t2-t1, t0:time to start missile t0-t1:time for collecting data 
				   //t1-t2:time to analyze data and interceptor setup t2:time to start interceptor
				   //t3:predicted time to intercept missile
	point vMissile;//velocity for missile at t1
	point pMissile;//position for missile at t1

	
	interceptor()
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		start.x=1500;	
		start.y=1500;
		start.z=0;
		v.x=-120;
		v.y=-60;
		v.z=pow(pow(vMax,2)-pow(v.x,2)-pow(v.y,2),0.5);
		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-3;
		on=1;
		//tStart=time(NULL);
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	interceptor(char c)
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		
		start.x=(rand()%500);	
		start.y=(rand()%500);
		start.z=0;
		v.x=(rand()%120);
		v.y=(rand()%120);
		v.z=pow(pow(vMax,2)-pow(v.x,2)-pow(v.y,2),0.5);
		g0=9.8;
		r0=(float) (rand()%100)/100;
		r1=(float) (rand()%100)/1000;
		r2=(float) (rand()%100)/10000;
		n0=(float) (rand()%10)/10-5;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	interceptor(point p0,point v0)
	{
		nVertex=1;
		shape=new point[1];
		shape[0]=start;
		start=p0;	
		v=v0;
		g0=9.8;
		r0=(float) (rand()%100)/100;
		r1=(float) (rand()%100)/1000;
		r2=(float) (rand()%100)/10000;
		n0=(float) (rand()%10)/10-5;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	interceptor(int f)//interceptor parameters from file 
	{
		FILE *fp;
		char fn[30];
		int i,j,n,temp; 
		id=f;
		sprintf(fn,"interceptor%d.txt",f);
		fp=fopen(fn,"r");
		if (fp==NULL) return;
		fscanf(fp,"%d",&nVertex);
		shape=new point[nVertex];
		neighbor=new int*[nVertex];
		
		
		for(i=0;i<nVertex;i++)
		{
			fscanf(fp,"%f %f %f",&(shape[i].x),&(shape[i].y),&(shape[i].z));
			neighbor[i]=new int[nVertex];
			for (j=0;j<nVertex;j++)
				neighbor[i][j]=0;
		}
		for(i=0;i<nVertex-1;i++)
		{
			fscanf(fp,"%d",&n);
			for (j=0;j<n;j++)
			{
				fscanf(fp,"%d",&temp);
				neighbor[i][temp]=1;
				neighbor[temp][i]=1;
			}
		}
		fscanf(fp,"%f",&n0);	
		fscanf(fp,"%f",&r0);	
		fscanf(fp,"%f",&r1);
		fscanf(fp,"%f",&r2);	
		fscanf(fp,"%f",&g0);
		fscanf(fp,"%f",&g1);
		fscanf(fp,"%f",&g2);	
		fscanf(fp,"%f %f %f",&(start.x),&(start.y),&(start.z));
		fscanf(fp,"%f %f %f",&(v.x),&(v.y),&(v.z));
		cout<<n0<<","<<r0<<","<<r1<<","<<r2<<","<<g0<<","<<endl;
		//tStart=time(NULL);
		on=1;
		current=start;
		//tCurrent=tStart;
		return;	
	}
	
	void fire(int m) //啟動資料分析 MISSILE 之 nCollect 筆資料 
	{
		statistics s(nCollect,m);
		s.accFn();
		for (int i=0;i<3;i++)
			xabc[i]=s.abc[i];
		return;
	}
	
	vector pNext(float t) //interceptor 於T時間後之位置 
	{
		vector vv(v);
		char tt[30];
		float r = (pow(vv.length,2)*r2 + vv.length*r1 + r0)/vv.length;
		//cout<<r<<endl;
		a.x=r*(-v.x)+n0*v.y/vv.length;
		a.y=r*(-v.y)+n0*(-v.x)/vv.length;
		a.z=r*(-v.z)-g0;
		current.x+=v.x*t+0.5*a.x*t*t;
		current.y+=v.y*t+0.5*a.y*t*t;
		current.z+=v.z*t+0.5*a.z*t*t;
		//tCurrent+=t;
		v.x=v.x+a.x*t;
		v.y=v.y+a.y*t;
		v.z=v.z+a.z*t;
		setcolor(15);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",v.x,v.y,v.z);
		outtextxy(wx/2+250,100*(id-1)+50,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",current.x,current.y,current.z);
		outtextxy(wx/2+250,100*(id-1)+30,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",a.x,a.y,a.z);
		outtextxy(wx/2+250,100*(id-1)+70,tt);	
		setcolor(color);
		return current;		
	}	
	
	vector xpNext(float t) //missile 於t時間後之預測位置 
	{
		vector vv(v);
		char tt[30];
		
		float temp=pow(tCount*tDelta+1,cAcc);
		a.x=xabc[0].x/temp + xabc[0].y;
		a.y=xabc[1].x/temp + xabc[1].y;
		a.z=xabc[2].x/temp + xabc[2].y;

		current.x+=v.x*t+0.5*a.x*t*t;
		current.y+=v.y*t+0.5*a.y*t*t;
		current.z+=v.z*t+0.5*a.z*t*t;
		//tCurrent+=t;
		v.x=v.x+a.x*t;
		v.y=v.y+a.y*t;
		v.z=v.z+a.z*t;
		setcolor(15);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",v.x,v.y,v.z);
		outtextxy(wx/2+250,100*(id-1)+50,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",current.x,current.y,current.z);
		outtextxy(wx/2+250,100*(id-1)+30,tt);
		sprintf(tt,"x:%8.3f y:%8.3f z:%8.3f ",a.x,a.y,a.z);
		outtextxy(wx/2+250,100*(id-1)+70,tt);	
		setcolor(color);
		return current;		
	}	
	
	void vInit() //initial velocity of interceptor  
	{

		point p1,v1,a1,p2=start,v2=v,a2;
		int i,j;
		int t1=nCollect,t2=nCollect+tSetup,t3=t2,tTemp=t1; 
		p1=pMissile;
		v1=vMissile;
		for (i=t1+1;i<=t2;i++) //compute p(t2) (missile)  
		{
			float temp=pow(i*tDelta+1,cAcc);
			a1.x=xabc[0].x/temp + xabc[0].y;
			a1.y=xabc[1].x/temp + xabc[1].y;
			a1.z=xabc[2].x/temp + xabc[2].y;
			p1.x+=v1.x*tDelta+0.5*a1.x*tDelta*tDelta;
			p1.y+=v1.y*tDelta+0.5*a1.y*tDelta*tDelta;
			p1.z+=v1.z*tDelta+0.5*a1.z*tDelta*tDelta;
			v1.x+=a1.x*tDelta;
			v1.y+=a1.y*tDelta;
			v1.z+=a1.z*tDelta;
		}
float minmin=dMax;
point vStart;
int tT=t2;
for (t3=t2+1;t3<t2+tSpan;t3++)	//compute p(t3) (missile) and p'(t3) (interceptor)
{
			//p2=start;
		float temp=pow(t3*tDelta+1,cAcc);
		a1.x=xabc[0].x/temp + xabc[0].y;
		a1.y=xabc[1].x/temp + xabc[1].y;
		a1.z=xabc[2].x/temp + xabc[2].y;
		p1.x+=v1.x*tDelta+0.5*a1.x*tDelta*tDelta;
		p1.y+=v1.y*tDelta+0.5*a1.y*tDelta*tDelta;
		p1.z+=v1.z*tDelta+0.5*a1.z*tDelta*tDelta;
		v1.x+=a1.x*tDelta;
		v1.y+=a1.y*tDelta;
		v1.z+=a1.z*tDelta;
		vector vTemp0(start,p1);
		if (vTemp0.length/vMax > (t3-t2)*tDelta || p1.z < 0) 
		{
			//cout<<"time not enough== t1: "<<t1<<",t2: "<<t2<<",t3: "<<t3<<", min time: "<<vTemp0.length/vMax<<", z: "<<p1.z<<endl;
			continue;
		}
		float angle1,angle2,dPre=dMax;
		vTemp0.target.z=0;
		angle1=n0*pow(t3-t2,2)/(2*vTemp0.length); //calibrated angle from p'(t2) to p(t3) 
	for(angle2=aStart;angle2<Pi/2;angle2=angle2+0.01) //elevated angle for interceptor
	{
		v2.z=vMax*sin(angle2);		
		v2.x=vTemp0.uv().rotate(3,angle1).scale(vMax*cos(angle2)).target.x;
		v2.y=vTemp0.uv().rotate(3,angle1).scale(vMax*cos(angle2)).target.y;
		point pV=v2;
		p2=start;
		for (i=t2+1;i<=t3;i++) //compute p(t3) (missile) and p'(t3) (interceptor)
		{
				vector vTemp(v2);
				float r = (vTemp.length)*r2 + r1 + r0/vTemp.length;
				a2.x=r*(-v2.x)+n0*v2.y/vTemp.length;
				a2.y=r*(-v2.y)+n0*(-v2.x)/vTemp.length;
				a2.z=r*(-v2.z)-g0;
				p2.x+=v2.x*tDelta+0.5*a2.x*tDelta*tDelta;
				p2.y+=v2.y*tDelta+0.5*a2.y*tDelta*tDelta;
				p2.z+=v2.z*tDelta+0.5*a2.z*tDelta*tDelta;
				v2.x+=a2.x*tDelta;
				v2.y+=a2.y*tDelta;
				v2.z+=a2.z*tDelta;
		}
		//getch();
		vector vTemp(p1,p2);
		if (vTemp.length >= dPre) 
		{
			if (dPre<minmin)
			{
				minmin=dPre;
				tT=t3;
				vStart=pV;
			}
				
			dPre=dMax;
			break;
		}
		dPre=vTemp.length;

		if (vTemp.length < dMin) 
		{
			cout<<t3<<": accepted distance= "<<vTemp.length<<endl;
			v=pV;
			return;
		}
	}
}
cout<<tT<<": distance= "<<minmin<<endl;//vTemp.length<<endl;
v=vStart;
		return;		
	}	
		
void show(int color) //於座標(x,y,z)畫出攔截飛彈 
{
	if (on==0) return;
	setcolor(color);
	point p2=shape[0];
	if (p2.x==0 && p2.y==0 && p2.z==0)
	{
		w.d3Point(current);
		return;
	}
	vector *temp=new vector[nVertex];	
	vector v1(v),v2(p2);
	int i,j;
	float a1=v1.angle(p2);
	v2=v2.rotate(2,a1);
	p2.x=v2.target.x;
	p2.y=v2.target.y;
	p2.z=0;
	v1.target.z=0;
	float a2=v1.angle(p2);
	v1=v1.outProduct(p2);
	if (v1.target.z>0) 
		a2=-a2;
	for (i=0;i<nVertex;i++)
	{
		temp[i].target=shape[i];
		temp[i]=temp[i].rotate(2,a1).rotate(3,a2).scale(SIZE).translate(current); 
	}
	for (i=0;i<nVertex;i++)
		for (j=0;j<i;j++)
		{
			if (neighbor[i][j]==1)
				w.d3LineSeg(temp[i].target,temp[j].target);
		}        
	return;
}
};

class firework //模擬爆炸之煙火  
{
	public:
	point *pCurrent;
	point *v;
	point *a;
	int *life;
	float g0,r0,r1,r2,n0;
	int *on;
	int nFire; //火花個數 
	view w;
	
	firework(int n) //設定 nFire 個火化之初速度,方向 及生命長度 (多久熄滅) 
	{
		int i;
		on=0;
		nFire=n;
		pCurrent=new point[n];
		v=new point[n];
		a=new point[n];
		life=new int[n];
		on=new int[n];
		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-1;
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
			vector vT(pT);
			v[i]=vT.uv().scale(rand()%50+20).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
	firework() //設定 nFire 個火化之初速度,方向 及生命長度 (多久熄滅) 
	{
		int i;
		int n=20+rand()%50;
		on=0;
		nFire=n;
		pCurrent=new point[n];
		v=new point[n];
		a=new point[n];
		life=new int[n];
		on=new int[n];
		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-1;
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
			vector vT(pT);
			v[i]=vT.uv().scale(rand()%50+20).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
	
	void pInit(point pp) //爆炸點 
	{
		int i;
		for (i=0;i<nFire;i++)
		{
			pCurrent[i]=pp;
			on[i]=1;
		}
		return;
	}
	
	void pNext(float t) //計算 T時間後之位置 
	{
		int i;
		float r;
		for (i=0;i<nFire;i++)
		{
			if (pCurrent[i].z>0 && life[i]>0)
				life[i]--;
			else 
			{
				on[i]=0;
				continue;
			}
			r = (pow(v[i].length(),2)*r2 + v[i].length()*r1 + r0)/v[i].length();
			a[i].x=r*(-v[i].x)+n0*v[i].y/v[i].length();
			a[i].y=r*(-v[i].y)+n0*(-v[i].x)/v[i].length();
			a[i].z=r*(-v[i].z)-g0;
			pCurrent[i].x+=v[i].x*t+0.5*a[i].x*t*t;
			pCurrent[i].y+=v[i].y*t+0.5*a[i].y*t*t;
			pCurrent[i].z+=v[i].z*t+0.5*a[i].z*t*t;
			v[i].x=v[i].x+a[i].x*t;
			v[i].y=v[i].y+a[i].y*t;
			v[i].z=v[i].z+a[i].z*t;
		}
		//getch();
		return;		
	}	
	
void show(int color) 
{
	int i;
	setcolor(color);
	
	for (i=0;i<nFire;i++)
	{
		if (on[i]==1)
			w.d3Point(pCurrent[i]);
	}
	return;
}
};



class terrain
{
	public:
	//float d;
	//missile m;
	terrain()
	{	
		srand (time(NULL));
		initwindow(wx1,wy1);
		setcolor(10);
		point p(fx,fy,fz),p1(0,0,0),d(1,0,0);
		float i;
		view w(p,p1,lFocus);
		//view w;
		p1.x=0;
		for (i=0;i<2*fy;i=i+200)
		{
			p1.y=i;
			w.d3LineEmit(p1,d);			
		}
		p1.y=0;
		d.x=0;
		d.y=1;
		for (i=0;i<2*fx;i=i+200)
		{
			p1.x=i;
			w.d3LineEmit(p1,d);			
		}
		p1.x=0;
		d.y=0;
		d.z=1;
		w.d3LineEmit(p1,d);	
		return;	
	}
	~terrain()
	{	
		getch();
		closegraph();
	}
};


void test1();
void fly();
void test_firework();
//void exfly();
int main()
{
	terrain t;
	//missile m(1);
	fly();
	//test_firework();
	//statistics s(nCollect);
	//s.showdata();
	//exfly();
	return 1;
}

void test_firework()
{
	firework *f;
	int i,tCount=100,n=rand()%10+10;
	f=new firework[n];
	for (i=0;i<n;i++)
	{
		point p(rand()%4000-2000,rand()%4000-2000,rand()%500+500);
		f[i].pInit(p);
	}
while (tCount>0)
{
	for (i=0;i<n;i++)
		f[i].show(i%5+10);	
	delay(10*lapse);
	for (i=0;i<n;i++)	
	{		
		f[i].show(0);
		f[i].pNext(tDelta);
	}
	tCount--;
}
	return;
}

void test1()
{
	point p1(200,800,100),p2(0,0,400),p3(0,0,0);
	vector v1(p1),v2(p2);
	int i;
	view w;

	float a=v1.angle(p2);
	w.d3LineSeg(p3,p1);
	getch();
	setcolor(10);
		w.d3LineSeg(p3,v2.target);
	v2=v2.rotate(2,a);
	getch();
		setcolor(11);
		w.d3LineSeg(p3,v2.target);
	p2.x=v2.target.x;
	p2.y=v2.target.y;
	p2.z=0;
	v1.target.z=0;
	a=v1.angle(p2);
	v2=v2.rotate(3,a);
	getch();
		setcolor(12);
		w.d3LineSeg(p3,v2.target);
	return;
}

	void fly()
	{
		vector v1,v2,v3,v4,v5; 
		missile m1(1),m3(2);//m3('r'); //create a missile with parameters in file missile1.txt and missile2.txt, respectively
		interceptor m2(1),m4(2),m5(2);//create a interceptor with parameters in file interceptor1.txt
		// interceptor m2 模擬 missile m1 的軌跡, 綠色為收集資料階段的軌跡, 粉紅色則為預估軌跡 
		firework f1(230),f2(200);
		FILE *fp1,*fp2;
		char tt[30];
	
		fp1=fopen("track1.txt","w");
		if (fp1==NULL) return;
		fp2=fopen("track2.txt","w");
		if (fp2==NULL) return;
		outtextxy(10,10,"missile:");
		outtextxy(10,30,"p:");
		outtextxy(10,50,"v:");
		outtextxy(10,70,"a:");
		outtextxy(wx/2+220,10,"interceptor:");
		outtextxy(wx/2+220,30,"p:");
		outtextxy(wx/2+220,50,"v:");
		outtextxy(wx/2+220,70,"a:");
		while(v1.target.z>=0 || v2.target.z>=0 || v3.target.z>=0) //missile 在地面上時顯示, 落地後則停止 
		{
			if (v1.target.z<0) m1.on=0; 
			m1.show(15);
			fprintf(fp1,"%f %f %f\n",m1.current.x,m1.current.y,m1.current.z); //收集 missile m1 之資料 (前 nCollect 筆) 
			if (tCount==nCollect) 
			{
				fclose(fp1); //資料收集完畢 
				m2.fire(1); //interceptor m2 對 missile m1 的 track1.txt  作分析 
				m4.fire(1); // interceptor m4 對 missile m1 的 track1.txt  作分析 
				m4.tSetup=nSetup; //interceptor 分析資料及準備發射時間 
				m4.pMissile=m1.current; //分析資料前之 missile m1 之位置 
				m4.vMissile=m1.v; // 分析資料前之 missile m1 之速度 
				m4.vInit(); //計算 interceptor 之初速度 (包括方向及速率 vMax) 
			}
			if (v2.target.z<0) m2.on=0;
			if (tCount>=nCollect)
				m2.show(13);
			else 
				m2.show(10);
			m4.show(10);
			if (v3.target.z<0) m3.on=0;
			m3.show(15);
			fprintf(fp2,"%f %f %f\n",m3.current.x,m3.current.y,m3.current.z);
			if (tCount==nCollect) 
			{
				fclose(fp2);
				m5.fire(2);
				m5.tSetup=nSetup;
				m5.pMissile=m3.current;
				m5.vMissile=m3.v;
				m5.vInit();
			}
			m5.show(10);
			setcolor(15);
			vector vT1(m1.current,m4.current);
			sprintf(tt,"Distance:%8.3f",vT1.length);
			outtextxy(wx/4+100,30,tt);
			vector vT2(m3.current,m5.current);
			sprintf(tt,"Distance:%8.3f",vT2.length);
			outtextxy(wx/4+100,130,tt);
			
			f1.show(10);
			f2.show(13);	
			delay(1000*lapse);			
			f1.show(0);
			f1.pNext(tDelta);
			f2.show(0);
			f2.pNext(tDelta);		
			m1.show(0);
			v1=m1.pNext(tDelta);
			m2.show(0);
			if (tCount>=nCollect)
				v2=m2.xpNext(tDelta);
			else 
				v2=m2.pNext(tDelta);
			m3.show(0);
			v3=m3.pNext(tDelta);
			if (tCount>=(nCollect+nSetup))
			{
				m4.show(0);
				v4=m4.pNext(tDelta);
				m5.show(0);
				v5=m5.pNext(tDelta);
			}
			 
			if (vT1.length<dMin)
			{
				m1.on=0;
				m2.on=0;
				m4.on=0;
				//getch();
				f1.pInit(m1.current);
			}
			if (vT2.length<dMin)
			{
				m3.on=0;
				m5.on=0;
				f2.pInit(m3.current);
				//getch();
			}
			tCount++;		       
		}
		return;
	}
	

