// arc-line.cpp : 定义控制台应用程序的入口点。
//

#include<iostream>
#include<vector>
#include<cmath>
#include<gl/glut.h>
using namespace std;

const static double PI=3.1415926;

const static double slot_width = 2.3;
const static double road_width = 8.0;
const static double tolerance = 0.3;
const static double ze = 4.5;

const static double car_width = 1.80;
const static double radius = 6;
const static double L1 = 0.7;
const static double L = 3.0;


struct point
{
	double x;
	double y;
};

class arc_line
{
private:
	point rotate_center;
	point intersection_point;
	point transition_point;
public:
	arc_line(point p, double theda)
	{
		rotate_center.x = p.x - radius*sin(theda);
		rotate_center.y = p.y + radius*cos(theda);
		intersection_point.x = -slot_width / 2;
		intersection_point.y = rotate_center.y - sqrt(pow(radius - car_width / 2, 2) - pow(rotate_center.x + slot_width / 2, 2));
		transition_point.y = rotate_center.y;
		transition_point.x = rotate_center.x + radius;
	}
	~arc_line(){}
	bool check_path1()
	{
		if (abs(transition_point.x) <= tolerance &&
			(transition_point.x - car_width) >= slot_width &&
			intersection_point.y < 0)
			return true;
		else return false;
	}

};

class line_arc_line
{
private:
	point rotate_center;
	point intersection_point;
	point transition_point1;
	double a, b;
public:
	line_arc_line(point p,double th)
	{
		a = tan(th);
		b = p.y - p.x*a;
		transition_point1.x = -radius + a*radius / sqrt(1 + a*a);
		transition_point1.y = -a*radius + b + a*a*radius / sqrt(1 + a*a);
		rotate_center.x = -radius;
		rotate_center.y = a*radius + b + radius / sqrt(1 + a*a);
		intersection_point.x = -slot_width / 2;
		intersection_point.y = rotate_center.y - sqrt(pow(radius - car_width / 2, 2) - pow(radius + slot_width / 2, 2));
	}
	bool check_path2()
	{
		if (intersection_point.y < 0)
			return true;
		else return false;
	}

};

class line_arc_arc_line
{
private:
	double tmin;
	double tmax;
	double dmin;
	double th;
public:
	point p;
	point p1;
	point p2;
	point p3;
	point pc;
	point pcc;
	line_arc_arc_line(point p, double th) :p(p), th(th), dmin(10000)
	{
		tmax = (p.y + road_width) / cos(th) - (L + L1) + car_width / (2 * tan(th));
		if (abs(-car_width*cos(th) + L1*sin(th) + p.x)>(slot_width / 2))
			tmin = p.y / sin(th) - L1 + car_width / (2 * tan(th));
		else tmin = (p.x + 0.5*slot_width) / cos(th) - L1 + car_width*tan(th) / 2;
	}

	double shortest_path()
	{
		for (double t = tmin; t <= tmax; t = t + 0.01)
		{
			if (pow(p.x + radius*sin(th) - radius + t*cos(th), 2) <= (4 * radius*radius) &&
				t*sin(th) + sqrt(4 * radius*radius - pow(p.x + radius*sin(th) - radius + t*cos(th), 2)) <=
				radius*cos(th) + sqrt(pow(radius - 0.5*car_width, 2) - pow(radius - 0.5*slot_width, 2)) - p.y)
			{
				/*p1.x = p.x + t*cos(th);
				p1.y = p.y + t*sin(th);
				pc.x = p1.x + radius*sin(th);
				pc.y = p1.y - radius*cos(th);
				pcc.x = radius;
				pcc.y = pc.y + sqrt(4 * radius*radius - (pc.x - radius)*(pc.x - radius));
				p2.x = 0.5*(pc.x + radius);
				p2.y = 0.5*(pc.y + pcc.y);
				p3.x = 0;
				p3.y = pcc.y;
				double alpha = 2 * asin(sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)) / (2 * radius));
				double beta = 2 * asin(sqrt(p2.x*p2.x + (p2.y - pcc.y)*(p2.y - pcc.y)) / (2 * radius));
				double d = t + radius*(alpha + beta) + (ze - pcc.y);*/
				double x1 = p.x + t*cos(th);
				double z1 = p.y + t*sin(th);
				double xc = x1 + radius*sin(th);
				double zc = z1 - radius*cos(th);
				double zcc = zc + sqrt(4 * radius*radius - (xc - radius)*(xc - radius));
				double x2 = 0.5*(xc + radius);
				double z2 = 0.5*(zc + zcc);
				double alpha = 2 * asin(sqrt((x1 - x2)*(x1 - x2) + (z1 - z2)*(z1 - z2)) / (2 * radius));
				double beta = 2 * asin(sqrt(x2*x2 + (z2 - zcc)*(z2 - zcc)) / (2 * radius));
				double d = t + radius*(alpha + beta) + (ze - zcc);
				if (d < dmin)
					dmin = d;
			}
		}
		return dmin;
	}

};


bool is_collision(point p,double th)
{
	vector<point> car_boundary;
	point car_point;
	double x,y;
	for ( x = -L1; x <= L+L1; x = x+0.01)
	{
		car_point.y = x*sin(th) + (-car_width / 2)*cos(th)+p.y;
		car_point.x = x*cos(th) - (-car_width / 2)*sin(th) + p.x;
		car_boundary.push_back(car_point);

		car_point.y = x*sin(th) + (+car_width / 2)*cos(th) + p.y;
		car_point.x = x*cos(th) - (+car_width / 2)*sin(th) + p.x;
		car_boundary.push_back(car_point);
	}
	for (y = -car_width / 2; y <= car_width / 2; y = y + 0.01)
	{
		car_point.x = (-L1)*cos(th)-y*sin(th)+p.x;
		car_point.y = (-L1)*sin(th) + y*cos(th) + p.y;
		car_boundary.push_back(car_point);

		car_point.x = (L1+L)*cos(th) - y*sin(th) + p.x;
		car_point.y = (L1 + L)*sin(th) + y*cos(th) + p.y;
		car_boundary.push_back(car_point);
	}
	for (auto po : car_boundary)
	{
		if ((po.x >= slot_width/2 && po.y >= 0) ||
			po.y<-road_width)
			return true;
	}
	return false;
}


point car_center;
double theda = 0.45*PI;
line_arc_arc_line laal(car_center, theda);

void myDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT);//GL_COLOR_BUFFER_BIT表示清除颜色
	glLineWidth(2);
	glBegin(GL_LINE_STRIP);
	glVertex2f(laal.p.x / 10, laal.p.y / 10);
	glVertex2f(laal.p1.x / 10, laal.p1.y / 10);
	glEnd();
	glLineWidth(2);
	glBegin(GL_LINE_STRIP);
	for (GLfloat x = laal.p1.x / 10; x >= laal.p2.x / 10; x = x - 0.01)
		glVertex2f(x, sqrt(radius*radius / 100 - (x - laal.pc.x / 10)*(x - laal.pc.x / 10)) + laal.pc.y / 10);
	glEnd();
	glLineWidth(2);
	glBegin(GL_LINE_STRIP);
	for (GLfloat x = laal.p2.x / 10; x >= 0; x = x - 0.01)
		glVertex2f(x, sqrt(radius*radius - (x - radius / 10)*(x - radius / 10)) + laal.pcc.y / 10);
	glEnd();
	glFlush(); //保证前面的OpenGL命令立即执行（而不是让它们在缓冲区中等待）。
}

int main(int argc, char *argv[])
{
	car_center.x = 2;
	car_center.y = -3;
	arc_line al(car_center,theda);
	line_arc_line lal(car_center, theda);
	
	bool get_feasible_path = false;
	if (al.check_path1() || lal.check_path2() || laal.shortest_path() < 10000.0)
		get_feasible_path = true;
	while (!get_feasible_path)
	{
		while (!is_collision(car_center,theda))
		{
			car_center.x = car_center.x + 0.1;
			car_center.y = car_center.y - 0.1;
			theda = theda - 0.5;
			if (al.check_path1() || lal.check_path2() || laal.shortest_path() < 10000.0)
				get_feasible_path = true;
		}
		car_center.x = car_center.x - 0.1;
		car_center.y = car_center.y - 0.1;
		theda = theda - 0.5;
		while (!is_collision(car_center,theda))
		{
			car_center.x = car_center.x - 0.1;
			car_center.y = car_center.y + 0.1;
			theda = theda - 0.5;
			if (al.check_path1() || lal.check_path2() || laal.shortest_path() < 10000.0)
				get_feasible_path = true;
		}
		car_center.x = car_center.x + 0.1;
		car_center.y = car_center.y + 0.1;
		while (!is_collision(car_center, theda))
		{
			car_center.x = car_center.x +0.1;
			car_center.y = car_center.y + 0.1;
			if (al.check_path1() || lal.check_path2() || laal.shortest_path() < 10000.0)
				get_feasible_path = true;
		}
		car_center.x = car_center.x - 0.1;
		car_center.y = car_center.y - 0.1;
		while (!is_collision(car_center, theda))
		{
			car_center.x = car_center.x - 0.1;
			car_center.y = car_center.y - 0.1;
			if (al.check_path1() || lal.check_path2() || laal.shortest_path() < 10000.0)
				get_feasible_path = true;
		}
		cout << "no available path" << endl;
		break;
	}
	if (get_feasible_path)
	{
		if (al.check_path1())
		{
			cout << "there is a availabel two-segments path " << endl;
		}
		else if (lal.check_path2())
			cout << "there is a availabel three-segments path" << endl;
		else 
		{ 
			cout << "there is a availabel four-segments path  " << laal.shortest_path() << endl; 
			/*glutInit(&argc, argv);//对GLUT进行初始化，这个函数必须在其它的GLUT使用之前调用一次
			glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE); //设置显示方式
			glutInitWindowPosition(100, 100); //设置窗口位置
			glutInitWindowSize(400, 400);//窗口大小
			glutCreateWindow("第一个OpenGL程序"); //根据前面设置的信息创建窗口。参数将被作为窗口的标题。
			glutDisplayFunc(&myDisplay); //当需要画图时，请调用myDisplay函数
			glutMainLoop(); //进行一个消息循环*/
		}
	}
	system("pause");
	return 0;
}

