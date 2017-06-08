#include "../headers/box.h"

box::box(bill::BillRBIntegrator algorithm, double _a, double _b, double _c, bill::vector position,
         bill::vector velocity, bill::quaternion rotation, bill::vector angular) : bill::BillRigidBody(algorithm,
                                                                                                       position,
                                                                                                       velocity,
                                                                                                       rotation,
                                                                                                       angular) {
    a = _a;
    b = _b;
    c = _c;

    colors.push_back(bill::vector({0.95686275, 0.34117647, 0.03921569}));
    colors.push_back(bill::vector({0.95686275, 0.34117647, 0.03921569}));
    colors.push_back(bill::vector({0.56470588, 0.6627451, 0.78039216}));
    colors.push_back(bill::vector({0.56470588, 0.6627451, 0.78039216}));
    colors.push_back(bill::vector({0.95294118, 0.75294118, 0.78431373}));
    colors.push_back(bill::vector({0.95294118, 0.75294118, 0.78431373}));

    radius = sqrt(0.25 * (a * a + b * b + c * c));

}

void box::rotatevector(bill::vector &v) {
    std::get<2>(present).rotateMe(v);
}

void box::rotatedcoordinates(bill::vector &i, bill::vector &j, bill::vector &k) {
    i = bill::vector({1., 0., 0.});
    j = bill::vector({0., 1., 0.});
    k = bill::vector({0., 0., 1.});

    rotatevector(i);
    rotatevector(j);
    rotatevector(k);
}

void box::vertices(std::vector<bill::vector> &vers) {
    vers.clear();

    bill::vector i, j, k;
    rotatedcoordinates(i, j, k);

    bill::vector x = std::get<0>(present);

    vers.push_back(x + 0.5 * (-a * i + b * j + c * k));
    vers.push_back(x + 0.5 * (a * i + b * j + c * k));
    vers.push_back(x + 0.5 * (a * i + b * j - c * k));
    vers.push_back(x + 0.5 * (-a * i + b * j - c * k));
    vers.push_back(x + 0.5 * (-a * i - b * j + c * k));
    vers.push_back(x + 0.5 * (a * i - b * j + c * k));
    vers.push_back(x + 0.5 * (a * i - b * j - c * k));
    vers.push_back(x + 0.5 * (-a * i - b * j - c * k));
}

void drawBBox(bill::vector min, bill::vector max);

void box::Draw() {
    std::vector<bill::vector> points;

    this->vertices(points);
    calculateBBox(points);
    drawBBox(minPoint, maxPoint);

    bill::GLaux::drawBox(points, colors, 1.);
}

double box::get_size(const unsigned int i) const {
    switch (i) {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        default:
            return -1.;
    }
} //zwraca wymiar prostopadłościaniu w kierunku: 0:x, 1:y, 2:z 


bill::vector box::get_versor(const unsigned int i) {
    bill::vector result({0., 0., 0.});
    result[i] = 1.;
    rotatevector(result);
    return result;
} // zwraca wersor tworzący wierzchołek: 0:x, 1:y, 2:z 


void box::get_vertices(std::vector<bill::vector> &vers) {
    vertices(vers);
} // zwraca std::vector wektorów do wierzchołków

void drawLine(bill::vector vector1, bill::vector vector2) {
    glPushMatrix();
    glLineWidth(2.5);
    glColor3f(1, 0.1, 0);
    glBegin(GL_LINES);
    glVertex3f(vector1[0], vector1[1], vector1[2]);
    glVertex3f(vector2[0], vector2[1], vector2[2]);
    glEnd();
    glPopMatrix();
}

void drawBBox(bill::vector min, bill::vector max) {
    drawLine(min, {max[0], min[1], min[2]}); //minx
    drawLine(min, {min[0], max[1], min[2]}); //miny
    drawLine(min, {min[0], min[1], max[2]}); //minz

    drawLine({max[0], min[1], min[2]}, {max[0], min[1], max[2]}); //minx maxy
    drawLine({max[0], min[1], min[2]}, {max[0], max[1], min[2]}); //minx maxz

    drawLine({min[0], max[1], min[2]}, {min[0], max[1], max[2]}); //miny maxx
    drawLine({min[0], max[1], min[2]}, {max[0], max[1], min[2]}); //miny maxz

    drawLine({min[0], min[1], max[2]}, {max[0], min[1], max[2]}); //minz maxy
    drawLine({min[0], min[1], max[2]}, {min[0], max[1], max[2]}); //minz maxx

    drawLine(max, {min[0], max[1], max[2]}); //maxx
    drawLine(max, {max[0], min[1], max[2]}); //maxy
    drawLine(max, {max[0], max[1], min[2]}); //maxz
}

void box::calculateBBox(std::vector<bill::vector> mesh) {
    double min_x, max_x,
           min_y, max_y,
           min_z, max_z;

    min_x = min_y = min_z = 100;
    max_x = max_y = max_z = -100;

    for (int i = 0; i < mesh.size(); i++) {
        if (mesh[i][0] < min_x) min_x = mesh[i][0];
        if (mesh[i][0] > max_x) max_x = mesh[i][0];
        if (mesh[i][1] < min_y) min_y = mesh[i][1];
        if (mesh[i][1] > max_y) max_y = mesh[i][1];
        if (mesh[i][2] < min_z) min_z = mesh[i][2];
        if (mesh[i][2] > max_z) max_z = mesh[i][2];
    }

    minPoint = {min_x, min_y, min_z};
    maxPoint = {max_x, max_y, max_z};
}

