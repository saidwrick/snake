#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

//camera variables
float cameraX = 0.0f, cameraY = 0.0f, cameraZ = 250.0f;
float cameraLookX=0, cameraLookY = 0, cameraLookZ = 0;
float cameraSpeed = 3;

// for animation
float xInc = 1.5;
float xIncCount = 0;
bool flip = false;

float snakeWidth = 15;
float snakeWidthTemp = 0;
float snakeHeight = 15;
float snakeHeightTemp = 0;

float uFactor = 0.1f;


struct Point2D {
    float x,y;
};

struct Point3D {
    float x,y,z;
};

std::vector<std::vector<Point3D>> points3D;

std::vector<Point2D> clickPoints;

std::vector<std::vector<float>> coEfs(4, std::vector<float>(2));

std::vector<std::vector<float>> coEfs3D(4, std::vector<float>(3));

std::vector<std::vector<Point3D>> headPoints;

// base control points for snake shape
std::vector < Point2D> points2D = {
    {110, 333},
    {174, 260},
    {241, 337},
    {304, 251},
    {371, 333},
    {430, 250},
    {482, 338}
};

// for animation
std::vector < Point2D> animatePoints = points2D;
std::vector < Point2D> startPoints = points2D;
std::vector < Point2D> endPoints;

void getCoEfs(Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
    coEfs[3][0] = (1.0f/6.0f) * (p0.x * (-1) + p1.x * (3) + p2.x * (-3) + p3.x);
    coEfs[2][0] = (1.0f/6.0f) * (p0.x*3 + p1.x*(-6) + p2.x*3);
    coEfs[1][0] = (1.0f/6.0f) * (p0.x*(-3) + p2.x*3);
    coEfs[0][0] = (1.0f/6.0f) * (p0.x + p1.x * 4 + p2.x);

    coEfs[3][1] = (1.0f/6.0f) * (p0.y * (-1) + p1.y * (3) + p2.y * (-3) + p3.y);
    coEfs[2][1] = (1.0f/6.0f) * (p0.y * 3 + p1.y * (-6) + p2.y * 3);
    coEfs[1][1] = (1.0f/6.0f) * (p0.y * (-3) + p2.y * 3);
    coEfs[0][1] = (1.0f/6.0f) * (p0.y + p1.y * 4 + p2.y);
}

void getCoEfs3D(Point3D p0, Point3D p1, Point3D p2, Point3D p3) {
    coEfs3D[3][0] = (1.0f/6.0f) * (p0.x * (-1) + p1.x * (3) + p2.x * (-3) + p3.x);
    coEfs3D[2][0] = (1.0f/6.0f) * (p0.x*3 + p1.x*(-6) + p2.x*3);
    coEfs3D[1][0] = (1.0f/6.0f) * (p0.x*(-3) + p2.x*3);
    coEfs3D[0][0] = (1.0f/6.0f) * (p0.x + p1.x * 4 + p2.x);

    coEfs3D[3][1] = (1.0f/6.0f) * (p0.y * (-1) + p1.y * (3) + p2.y * (-3) + p3.y);
    coEfs3D[2][1] = (1.0f/6.0f) * (p0.y * 3 + p1.y * (-6) + p2.y * 3);
    coEfs3D[1][1] = (1.0f/6.0f) * (p0.y * (-3) + p2.y * 3);
    coEfs3D[0][1] = (1.0f/6.0f) * (p0.y + p1.y * 4 + p2.y);

    coEfs3D[3][2] = (1.0f/6.0f) * (p0.z * (-1) + p1.z * (3) + p2.z * (-3) + p3.z);
    coEfs3D[2][2] = (1.0f/6.0f) * (p0.z * 3 + p1.z * (-6) + p2.z * 3);
    coEfs3D[1][2] = (1.0f/6.0f) * (p0.z * (-3) + p2.z * 3);
    coEfs3D[0][2] = (1.0f/6.0f) * (p0.z + p1.z * 4 + p2.z);
}

// get mid points between two points
Point3D getMidPoint(Point3D p1, Point3D p2) {
    Point3D mid = {(p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2};
    return mid;
}

//gets the initial b-spline curve co-ords and processes each point to create the U's
void getSegmentPoints() {
    for (float u = 0; u <= 1; u += uFactor) {
        float x = coEfs[0][0] + coEfs[1][0] * u + coEfs[2][0] * u * u + coEfs[3][0] * u * u * u;
        float y = coEfs[0][1] + coEfs[1][1] * u + coEfs[2][1] * u * u + coEfs[3][1] * u * u * u;

        // for each x,y point on the curve, get the 3D control points for the upside down U shape of the snake body
        std::vector <Point3D> controlPoints;
        std::vector <Point3D> uPoints;
        Point3D p1 = { x,y,0 };
        Point3D plast = { x,y + snakeWidthTemp,0 };
        // pmid is p5
        Point3D pmid = { (p1.x + plast.x) / 2, (p1.y + plast.y) / 2, snakeHeightTemp };

        Point3D p3 = getMidPoint(p1, pmid);
        Point3D p2 = getMidPoint(p1, p3);
        Point3D p4 = getMidPoint(p3, pmid);

        Point3D p7 = getMidPoint(pmid, plast);
        Point3D p6 = getMidPoint(pmid, p7);
        Point3D p8 = getMidPoint(p7, plast);

        // can use all points, but body is more triangular
        // p1 and plast twice to include them in the curve
        controlPoints.insert(controlPoints.end(), { p1,p1,p3,pmid,p7,plast,plast });

        for (int i = 0; i < controlPoints.size() - 3; i++) {
            getCoEfs3D(controlPoints[i], controlPoints[i + 1], controlPoints[i + 2], controlPoints[i + 3]);

            for (float u = 0; u <= 1; u += uFactor) {
                float x = coEfs3D[0][0] + coEfs3D[1][0] * u + coEfs3D[2][0] * u * u + coEfs3D[3][0] * u * u * u;
                float y = coEfs3D[0][1] + coEfs3D[1][1] * u + coEfs3D[2][1] * u * u + coEfs3D[3][1] * u * u * u;
                float z = coEfs3D[0][2] + coEfs3D[1][2] * u + coEfs3D[2][2] * u * u + coEfs3D[3][2] * u * u * u;

                uPoints.push_back({ x,y,z });

            }
        }
        points3D.push_back(uPoints);
        if (snakeWidthTemp < snakeWidth) {
            snakeWidthTemp+=0.5;
        }
        if (snakeHeightTemp < snakeHeight) {
            snakeHeightTemp +=1;
        }
    }
}

// takes last U shape and draws a cone shape to the mid point using the same idea as the main body
void getHeadPoints() {
    std::vector <Point3D> lastU = points3D[points3D.size()-1];
    for (int i = 0; i < lastU.size(); i++) {
    // make U's
        Point3D first = {lastU[0].x, lastU[0].y, lastU[0].z};
        Point3D last={lastU[lastU.size()-1].x, lastU[lastU.size()-1].y, 0 };
        Point3D pMidU=getMidPoint(first, last);       

        float dirX = lastU[0].x - points3D[points3D.size()-2][0].x;
        //dirX = dirX/abs(dirX);
        float dirY = lastU[0].y - points3D[points3D.size()-2][0].y;
        //dirY = dirY/abs(dirY);

        if (dirX > 0) {
            pMidU.x += 25;
        }
        else {
            pMidU.x-=25;
        }

        Point3D p1U = {lastU[i].x, lastU[i].y, lastU[i].z};
        Point3D p2U = getMidPoint(p1U, pMidU);
        Point3D p3U = getMidPoint(p1U, p2U);

        std::vector <Point3D> controlPointsU = {p1U, p1U,p3U, pMidU, pMidU};
        std::vector <Point3D> headPointsU;
        headPointsU.push_back(p1U);

        for (int i = 0; i < controlPointsU.size() - 3; i++) {
            getCoEfs3D(controlPointsU[i], controlPointsU[i + 1], controlPointsU[i + 2], controlPointsU[i + 3]);

            for (float u = 0; u <= 1; u += uFactor) {
                float x = coEfs3D[0][0] + coEfs3D[1][0] * u + coEfs3D[2][0] * u * u + coEfs3D[3][0] * u * u * u;
                float y = coEfs3D[0][1] + coEfs3D[1][1] * u + coEfs3D[2][1] * u * u + coEfs3D[3][1] * u * u * u;
                float z = coEfs3D[0][2] + coEfs3D[1][2] * u + coEfs3D[2][2] * u * u + coEfs3D[3][2] * u * u * u;

                headPointsU.push_back({x,y,z});
            }
            headPoints.push_back(headPointsU);
        }
    }
}

// draws a b-spline curve from initial starting co-ords
// expands each point by width and then connects the two in a 3D upside down U shape (another b-spline curve)
// connect all the points to get the snake body
void getSnakePoints() {
    // so line passes through first and last point
    Point2D front = points2D.front();
    Point2D back = points2D.back();
    points2D.insert(points2D.begin(), front);
    points2D.push_back(back);

    // get 2d curve and process each point 
    snakeWidthTemp = 0;
    snakeHeightTemp = 0;
    for (int i = 0; i < points2D.size() - 3; i++) {
        getCoEfs(points2D[i], points2D[i + 1], points2D[i + 2], points2D[i + 3]);
        getSegmentPoints();
    }
    getHeadPoints();
}

void getAnimationCoords() {
    // find the midline between current point and next point and reflect Y over the mid line to get the end point of movement
    float midY;
    for (int i =0; i < points2D.size()-1; i++) {
        midY = (points2D[i].y + points2D[i+1].y)/2;
        float diff = points2D[i].y - midY;
        float y = midY-diff;
        endPoints.push_back({points2D[i].x,y});
    }
    // add last point using previous point to calculate midline
    float diff = points2D[points2D.size()-1].y - midY;
    float y = midY-diff;
    endPoints.push_back({points2D[points2D.size()-1].x, y});
}

// mirrors the co-ordinates of original snake shape for end shape, gradually transitions from start to end to animate
void animate() {
    // speed of animation
    float step = 25;

    // alternates between start and end shapes
    if (!flip) {
        for (int i = 0; i < animatePoints.size(); i++) {
            float inc = (startPoints[i].y - endPoints[i].y)/step;
            animatePoints[i].y += inc;
            animatePoints[i].x += xInc;
        }
        if ((startPoints[1].y - endPoints[1].y) > 0) {
            if (animatePoints[1].y >= startPoints[1].y) {
                flip = true;
            }
        }
        else {
            if (animatePoints[1].y <= startPoints[1].y) {
                flip = true;
            }
        }
    }
    else {
        for (int i = 0; i < animatePoints.size(); i++) {
            float inc = (startPoints[i].y - endPoints[i].y) / step;
            animatePoints[i].y -= inc;
            animatePoints[i].x += xInc;
        }
        if ((startPoints[1].y - endPoints[1].y) > 0) {
            if (animatePoints[1].y <= endPoints[1].y) {
                flip = false;
            }
        }
        else {
            if (animatePoints[1].y >= endPoints[1].y) {
                flip = false;
            }
        }

    }
    points2D = animatePoints;
    points3D.clear();
    headPoints.clear();
    getSnakePoints();
}

// remove cameraLook if you want to rotate the snake
void keyPress(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        cameraY += cameraSpeed;
        cameraLookY += cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        cameraY -= cameraSpeed;
        cameraLookY -= cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        cameraX -= cameraSpeed;
        cameraLookX -= cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        cameraX += cameraSpeed;
        cameraLookX += cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        cameraZ += cameraSpeed;
        cameraLookZ += cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        cameraZ -= cameraSpeed;
        cameraLookZ -= cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
        clickPoints.clear();
        animate();
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        exit(0);
    }
}

// draw user click points
void drawPoints() {
    glColor3f(0,0,1);
    glPointSize(5);
    glBegin(GL_POINTS);
    for (auto p : clickPoints) {
        glVertex3f(p.x,p.y,0);
    }
    glEnd();
}

void drawSnake() {
    glColor3f(1, 0, 0);
    // draw each U
    for (auto uPoints : points3D) {
        glBegin(GL_LINE_STRIP);
        for (Point3D p : uPoints) {
            glVertex3f(p.x, p.y, p.z);
        }
        // connect the bottom;
        glVertex3f(uPoints[0].x, uPoints[0].y, uPoints[0].z);
        glEnd();
    }

    // connect the U's
    for (int i = 0; i < points3D[0].size(); i++) {
        glBegin(GL_LINE_STRIP);
        for (auto uPoints : points3D) {
            glVertex3f(uPoints[i].x, uPoints[i].y, uPoints[i].z);
        }
        glEnd();
    }

    // draw head
    for (auto uPoints : headPoints) {
        glBegin(GL_LINE_STRIP);
        for (Point3D p : uPoints) {
            glVertex3f(p.x, p.y, p.z);
        }
        //connect the bottom;
        glVertex3f(uPoints[0].x, uPoints[0].y, uPoints[0].z);
        glEnd();
    }

    // connect the U's
    for (int i = 0; i < headPoints[0].size(); i++) {
        glBegin(GL_LINE_STRIP);
        for (auto uPoints : headPoints) {
            glVertex3f(uPoints[i].x, uPoints[i].y, uPoints[i].z);
        }
        glEnd();
    }
}

//convert mouse click in 2d to 3d. NOT MY CODE
Point2D get3DCoordinates(GLFWwindow* window, double mouseX, double mouseY) {
    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    GLdouble nearX, nearY, nearZ, farX, farY, farZ;

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    mouseY = viewport[3] - mouseY;

    gluUnProject(mouseX, mouseY, 0.0, modelview, projection, viewport, &nearX, &nearY, &nearZ);
    gluUnProject(mouseX, mouseY, 1.0, modelview, projection, viewport, &farX, &farY, &farZ);

    double dirX = farX - nearX;
    double dirY = farY - nearY;
    double dirZ = farZ - nearZ;

    if (dirZ == 0) {
        cout << "Ray is parallel to the plane z=0, no intersection!\n";
    }

    double t = -nearZ / dirZ;
    double x = nearX + t * dirX;
    double y = nearY + t * dirY;


    Point2D p = {x, y};
    return (p);
}

// update camera
void updateCameraView() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(cameraX, cameraY, cameraZ,
        cameraLookX, cameraLookY, cameraLookZ,
        0.0f, 1.0f, 0.0f); 
}

// set initial camera to middle of snake
void setCamera() {
    int mid = int(points3D.size()/2);
    cameraX = points3D[mid][0].x;
    cameraY = points3D[mid][0].y;

    cameraLookX = cameraX;
    cameraLookY = cameraY;
}

// get points on mouse click 
void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        // get 3D co-ords
        Point2D p = get3DCoordinates(window,xpos, ypos);
        clickPoints.push_back(p);

    }

    // draw snake on right click
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        points3D.clear();
        headPoints.clear();
        points2D = clickPoints;
        animatePoints = points2D;
        startPoints = points2D;
        endPoints.clear();
        getAnimationCoords();

        flip = true;
        getSnakePoints();
        setCamera();
    }
}

int main() {
    cout << "Controls: WASD to move the camera\n Z = zoom in, X = zoom out\n hold M to animate \n Q to quit \n left click to add control points \n right click to draw new snake using those points";
    getAnimationCoords();
    getSnakePoints();
    setCamera();

    if (!glfwInit()) {
        std::cerr << "GLFW Initialization failed!" << std::endl;
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(1400, 600, "proj2", NULL, NULL);
    if (!window) {
        std::cerr << "GLFW Window creation failed!" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glEnable(GL_DEPTH_TEST); 

    glMatrixMode(GL_PROJECTION);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glLoadIdentity();
    gluPerspective(45.0f, 1400.0f / 600.0f, 0.1f, 500.0f);

    glfwSetMouseButtonCallback(window, mouseClick);
    glfwSetKeyCallback(window, keyPress);

    // main loop
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        updateCameraView();

        glPushMatrix();
        glClear(GL_COLOR_BUFFER_BIT);
        drawSnake();
        drawPoints();
        glFlush();
        glPopMatrix();

        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
