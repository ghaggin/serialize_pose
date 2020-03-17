#include <cassert>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <serialize_pose.hpp>

using namespace Eigen;

int main() {
    SerializePose sp;
    Vector3f      pos(1, 2, 3);
    Quaternionf   att(1, 2, 3, 4);

    sp.serialize(pos, att);

    constexpr size_t size = sizeof(SerializePose::pose_packet_t) + 4;
    uint8_t          msg[size];

    sp.getPacket(msg, size);

    SerializePose sp2;
    sp2.deserialize(msg, size);

    auto pos2 = sp2.getPos();
    auto att2 = sp2.getAtt();

    std::cout << pos2.x() << ' ' << pos2.y() << ' ' << pos2.z() << std::endl;
    std::cout << att2.w() << ' ' << att2.x() << ' ' << att2.y() << ' ' << att2.z() << std::endl;

    return 0;
}