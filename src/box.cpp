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

void drawWire(std::vector<bill::vector> vertecies);
std::vector<bill::vector> calculateBoundingBox(std::vector<bill::vector> &vers);
//void draw_bbox(std::vector<bill::vector> mesh);

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

void box::Draw() {
    std::vector<bill::vector> points;
    std::vector<bill::vector> wire;
    this->vertices(points);
    wire = calculateBoundingBox(points);
    draw_bbox(wire);
//    draw_bbox(points);

    bill::GLaux::drawBox(points, colors, 1.);
//    drawWire(wire);
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

std::vector<bill::vector> getMaxFromTop(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> maxVecs;
    double maxY = vers[0][1];
    for (int i = 0; i < 4; i++) {
        if (maxY < vers[i][1]) {
            maxY = vers[i][1];
        }
    }

    for (int i = 0; i < 4; i++) {
        maxVecs.push_back({vers[i][0], maxY, vers[i][2]});
    }

    return maxVecs;
}

std::vector<bill::vector> getMinFromBottom(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> minVecs;
    double minY = vers[0][1];
    for (int i = 4; i < 8; i++) {
        if (minY > vers[i][1]) {
            minY = vers[i][1];
        }
    }

    for (int i = 4; i < 8; i++) {
        minVecs.push_back({vers[i][0], minY, vers[i][2]});
    }

    return minVecs;
}

std::vector<bill::vector> getMinFromLeft(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> minVecs;
    double minZ = vers[0][2];
    if (minZ > vers[0][2]) {
        minZ = vers[0][2];
    }
    if (minZ > vers[3][2]) {
        minZ = vers[3][2];
    }
    if (minZ > vers[4][2]) {
        minZ = vers[4][2];
    }
    if (minZ > vers[7][2]) {
        minZ = vers[7][2];
    }

    minVecs.push_back({vers[0][0], vers[0][1], minZ});
    minVecs.push_back({vers[3][0], vers[3][1], minZ});
    minVecs.push_back({vers[4][0], vers[4][1], minZ});
    minVecs.push_back({vers[7][0], vers[7][1], minZ});

    return minVecs;
}

std::vector<bill::vector> getMaxFromRight(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> maxVecs;
    double maxZ = vers[1][2];
    if (maxZ < vers[1][2]) {
        maxZ = vers[1][2];
    }
    if (maxZ < vers[2][2]) {
        maxZ = vers[2][2];
    }
    if (maxZ < vers[5][2]) {
        maxZ = vers[5][2];
    }
    if (maxZ < vers[6][2]) {
        maxZ = vers[6][2];
    }

    maxVecs.push_back({vers[1][0], vers[1][1], maxZ});
    maxVecs.push_back({vers[2][0], vers[2][1], maxZ});
    maxVecs.push_back({vers[5][0], vers[5][1], maxZ});
    maxVecs.push_back({vers[6][0], vers[6][1], maxZ});

    return maxVecs;
}

std::vector<bill::vector> getMinFromFront(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> minVecs;
    double minX = vers[0][0];
    if (minX > vers[0][0]) {
        minX = vers[0][0];
    }
    if (minX > vers[1][0]) {
        minX = vers[1][0];
    }
    if (minX > vers[4][0]) {
        minX = vers[4][0];
    }
    if (minX > vers[5][0]) {
        minX = vers[5][0];
    }

    minVecs.push_back({minX, vers[0][1], vers[0][2]});
    minVecs.push_back({minX, vers[1][1], vers[1][2]});
    minVecs.push_back({minX, vers[4][1], vers[4][2]});
    minVecs.push_back({minX, vers[5][1], vers[5][2]});

    return minVecs;
}

std::vector<bill::vector> getMaxFromBack(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> maxVecs;
    double maxX = vers[0][0];
    if (maxX < vers[2][0]) {
        maxX = vers[2][0];
    }
    if (maxX < vers[3][0]) {
        maxX = vers[3][0];
    }
    if (maxX < vers[6][0]) {
        maxX = vers[6][0];
    }
    if (maxX < vers[7][0]) {
        maxX = vers[7][0];
    }

    maxVecs.push_back({maxX, vers[2][1], vers[2][2]});
    maxVecs.push_back({maxX, vers[3][1], vers[3][2]});
    maxVecs.push_back({maxX, vers[6][1], vers[6][2]});
    maxVecs.push_back({maxX, vers[7][1], vers[7][2]});

    return maxVecs;
}

std::vector<bill::vector> calculateBoundingBox(std::vector<bill::vector> &vers) {
    std::vector<bill::vector> vectors;
    std::vector<bill::vector> vectorsfromFunction;

    vectorsfromFunction = getMaxFromTop(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    vectorsfromFunction = getMinFromBottom(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    vectorsfromFunction = getMinFromLeft(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    vectorsfromFunction = getMaxFromRight(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    vectorsfromFunction = getMinFromFront(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    vectorsfromFunction = getMaxFromBack(vers);
    for(bill::vector vec : vectorsfromFunction) {
        vectors.push_back(vec);
    }

    return vectors;
}

void drawLine(bill::vector vector1, bill::vector vector2);

void drawWire(std::vector<bill::vector> vertecies){

    // top
    drawLine(vertecies[0], vertecies[1]);
    drawLine(vertecies[1], vertecies[2]);
    drawLine(vertecies[2], vertecies[3]);
    drawLine(vertecies[3], vertecies[0]);

    drawLine(vertecies[4], vertecies[5]);
    drawLine(vertecies[5], vertecies[6]);
    drawLine(vertecies[6], vertecies[7]);
    drawLine(vertecies[7], vertecies[4]);

    drawLine(vertecies[0], vertecies[3]);
    drawLine(vertecies[3], vertecies[7]);
    drawLine(vertecies[7], vertecies[4]);
    drawLine(vertecies[4], vertecies[0]);

    drawLine(vertecies[1], vertecies[2]);
    drawLine(vertecies[2], vertecies[6]);
    drawLine(vertecies[6], vertecies[5]);
    drawLine(vertecies[5], vertecies[1]);

    drawLine(vertecies[0], vertecies[1]);
    drawLine(vertecies[1], vertecies[5]);
    drawLine(vertecies[5], vertecies[4]);
    drawLine(vertecies[4], vertecies[0]);

    drawLine(vertecies[3], vertecies[2]);
    drawLine(vertecies[2], vertecies[6]);
    drawLine(vertecies[6], vertecies[7]);
    drawLine(vertecies[7], vertecies[3]);
}

void drawLine(bill::vector vector1, bill::vector vector2) {
    glPushMatrix();
    glLineWidth(2.5);
    glColor3f(0.8, 0.5, 0.8);
    glBegin(GL_LINES);
    glVertex3f(vector1[0], vector1[1], vector1[2]);
    glVertex3f(vector2[0], vector2[1], vector2[2]);
    glEnd();
    glPopMatrix();
}

void box::draw_bbox(std::vector<bill::vector> mesh) {
    double min_x, max_x,
           min_y, max_y,
           min_z, max_z;
    min_x = max_x = mesh[0][0];
    min_y = max_y = mesh[0][0];
    min_z = max_z = mesh[0][0];
    for (int i = 0; i < mesh.size(); i++) {
        if (mesh[i][0] < min_x) min_x = mesh[i][0];
        if (mesh[i][0] > max_x) max_x = mesh[i][0];
        if (mesh[i][1] < min_y) min_y = mesh[i][1];
        if (mesh[i][1] > max_y) max_y = mesh[i][1];
        if (mesh[i][2] < min_z) min_z = mesh[i][2];
        if (mesh[i][2] > max_z) max_z = mesh[i][2];
    }
    bill::vector min = {min_x, min_y, min_z};
    bill::vector max = {max_x, max_y, max_z};
//    bill::vector size = {max_x-min_x, max_y-min_y, max_z-min_z};
//    bill::vector center = {(min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2};

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

