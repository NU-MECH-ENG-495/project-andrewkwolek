// MESSAGE CAMERA_IMAGE_CAPTURED support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CAMERA_IMAGE_CAPTURED message
 *
 * Information about a captured image. This is emitted every time a message is captured.
        MAV_CMD_REQUEST_MESSAGE can be used to (re)request this message for a specific sequence number or range of sequence numbers:
        MAV_CMD_REQUEST_MESSAGE.param2 indicates the sequence number the first image to send, or set to -1 to send the message for all sequence numbers.
        MAV_CMD_REQUEST_MESSAGE.param3 is used to specify a range of messages to send:
        set to 0 (default) to send just the the message for the sequence number in param 2,
        set to -1 to send the message for the sequence number in param 2 and all the following sequence numbers,
        set to the sequence number of the final message in the range.
 */
struct CAMERA_IMAGE_CAPTURED : mavlink::Message {
    static constexpr msgid_t MSG_ID = 263;
    static constexpr size_t LENGTH = 255;
    static constexpr size_t MIN_LENGTH = 255;
    static constexpr uint8_t CRC_EXTRA = 133;
    static constexpr auto NAME = "CAMERA_IMAGE_CAPTURED";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint64_t time_utc; /*< [us] Timestamp (time since UNIX epoch) in UTC. 0 for unknown. */
    uint8_t camera_id; /*<  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id). Field name is usually camera_device_id. */
    int32_t lat; /*< [degE7] Latitude where image was taken */
    int32_t lon; /*< [degE7] Longitude where capture was taken */
    int32_t alt; /*< [mm] Altitude (MSL) where image was taken */
    int32_t relative_alt; /*< [mm] Altitude above ground */
    std::array<float, 4> q; /*<  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0) */
    int32_t image_index; /*<  Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1) */
    int8_t capture_result; /*<  Boolean indicating success (1) or failure (0) while capturing this image. */
    std::array<char, 205> file_url; /*<  URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  time_utc: " << time_utc << std::endl;
        ss << "  camera_id: " << +camera_id << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  relative_alt: " << relative_alt << std::endl;
        ss << "  q: [" << to_string(q) << "]" << std::endl;
        ss << "  image_index: " << image_index << std::endl;
        ss << "  capture_result: " << +capture_result << std::endl;
        ss << "  file_url: \"" << to_string(file_url) << "\"" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_utc;                      // offset: 0
        map << time_boot_ms;                  // offset: 8
        map << lat;                           // offset: 12
        map << lon;                           // offset: 16
        map << alt;                           // offset: 20
        map << relative_alt;                  // offset: 24
        map << q;                             // offset: 28
        map << image_index;                   // offset: 44
        map << camera_id;                     // offset: 48
        map << capture_result;                // offset: 49
        map << file_url;                      // offset: 50
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_utc;                      // offset: 0
        map >> time_boot_ms;                  // offset: 8
        map >> lat;                           // offset: 12
        map >> lon;                           // offset: 16
        map >> alt;                           // offset: 20
        map >> relative_alt;                  // offset: 24
        map >> q;                             // offset: 28
        map >> image_index;                   // offset: 44
        map >> camera_id;                     // offset: 48
        map >> capture_result;                // offset: 49
        map >> file_url;                      // offset: 50
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
