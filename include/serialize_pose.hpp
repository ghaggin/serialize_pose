#pragma once

#include <eigen3/Eigen/Dense>

#include <array>
#include <cstring>

class SerializePose {
  public:
    struct __attribute__((packed)) pose_packet_t {
        double time;
        float  x, y, z, qx, qy, qz, qw;
    };

  public:
    SerializePose() {
        /** fill first two start bytes */
        data_packet_[0] = start_byte1;
        data_packet_[1] = start_byte2;
    }

    /** Builds serialized data packet from eigen vecotor and quaternion */
    void serialize(Eigen::Vector3f v, Eigen::Quaternionf q) {
        eigen_pos_ = v;
        eigen_att_ = q;
        buildDataPacket();
    }

    /** Deserializes data packet into eigen vector and quaternion */
    bool deserialize(uint8_t* const in_packet, const size_t size) {
        if (size != data_packet_size_) {
            throw std::runtime_error("Data packet size incorrect");
        }

        /** Check that the start bytes match */
        if (in_packet[0] != start_byte1 || in_packet[1] != start_byte2)
            return false;

        /** copy the data from in_packet to data_packet_ */
        memcpy(data_packet_, in_packet, data_packet_size_);

        /** verify that the checksum matches
         *  return false if it does not
         */
        if (!checkFletcher16())
            return false;

        /** Convert the data packet to eigen */
        decomposePacket();
    }

    /** Return eigen vector */
    Eigen::Vector3f getPos() const {
        return eigen_pos_;
    }

    /** return eigen quaternion */
    Eigen::Quaternionf getAtt() const {
        return eigen_att_;
    }

    /** Copies data from packet into external array (out_packet) */
    bool getPacket(uint8_t* const out_packet, const size_t size) const {
        if (size != data_packet_size_) {
            throw std::runtime_error("Data packet size incorrect");
        }

        /** Verify checksum prior to sending out packet */
        if (!checkFletcher16()) {
            return false;
        }

        /** copy the data and return true */
        memcpy(out_packet, data_packet_, data_packet_size_);
        return true;
    }

    /** returns data packet size determined from start bytes,
     * check sum and pose packet size
     */
    size_t getPacketSize() const {
        return data_packet_size_;
    }

  private:
    /**
     * Builds the data packet from the eigen varaibles
     */
    void buildDataPacket() {
        pose_packet_t pose_packet;
        pose_packet.x  = eigen_pos_.x();
        pose_packet.y  = eigen_pos_.y();
        pose_packet.z  = eigen_pos_.z();
        pose_packet.qx = eigen_att_.x();
        pose_packet.qy = eigen_att_.y();
        pose_packet.qz = eigen_att_.z();
        pose_packet.qw = eigen_att_.w();

        /** Copy data pose packet into data packet */
        memcpy(data_start_, &pose_packet, pose_packet_size_);

        /** Calculate and append the fletcher16 checksum to the data packet */
        addFletcher16();
    }

    /**
     * Fills out Eigen member variables from data packet
     * call this after receiving new serialized data
     */
    void decomposePacket() {
        pose_packet_t pose_packet;
        memcpy(&pose_packet, data_start_, pose_packet_size_);
        eigen_att_.x() = pose_packet.qx;
        eigen_att_.y() = pose_packet.qy;
        eigen_att_.z() = pose_packet.qz;
        eigen_att_.w() = pose_packet.qw;

        eigen_pos_.x() = pose_packet.x;
        eigen_pos_.y() = pose_packet.y;
        eigen_pos_.z() = pose_packet.z;
    }

    /**
     * Calculates Fletcher 16 CRC from the data part of message
     */
    uint16_t calcFletcher16(uint8_t* const buf, const size_t buflen) const {
        // Compute Fletcher16 CRC
        uint8_t chksm0 = 0, chksm1 = 0;
        for (size_t i = 0; i < buflen; ++i) {
            chksm0 += buf[i];
            chksm1 += chksm0;
        }

        return (uint16_t)((chksm1 << 8) | chksm0);
    }

    /**
     * Calculates check sum and fills the last two bytes of the data
     * packet
     */
    uint16_t addFletcher16() {
        // Compute Fletcher16 CRC
        uint16_t retVal = calcFletcher16(data_start_, pose_packet_size_);

        // Copy the value to 'dest'
        memcpy(data_end_, (char*)&retVal, 2);

        return retVal;
    }

    /**
     * Checks the checksum of the data packet, returns false
     * if the checksum does not match, true otherwise
     */
    bool checkFletcher16() const {
        uint16_t check_sum1 = calcFletcher16(data_start_, pose_packet_size_);
        uint16_t check_sum2;
        memcpy(&check_sum2, data_end_, 2);
        if (check_sum1 == check_sum2) {
            return true;
        }
        return false;
    }

    /**
     * Data sizes and start bytes
     */
    static constexpr size_t  pose_packet_size_ = sizeof(pose_packet_t);
    static constexpr size_t  data_packet_size_ = pose_packet_size_ + 4;
    static constexpr uint8_t start_byte1       = 0x81;
    static constexpr uint8_t start_byte2       = 0xA1;

    /** Data packet */
    uint8_t        data_packet_[data_packet_size_];
    uint8_t* const data_start_ = data_packet_ + 2;
    uint8_t* const data_end_   = data_start_ + pose_packet_size_;

    /** Eigen Data */
    Eigen::Vector3f    eigen_pos_;
    Eigen::Quaternionf eigen_att_;
};