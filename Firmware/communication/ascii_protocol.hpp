#ifndef __ASCII_PROTOCOL_HPP
#define __ASCII_PROTOCOL_HPP


#define MAX_LINE_LENGTH ((size_t)256)

class AsciiProtocol {
public:
    AsciiProtocol() {}

    void start();

private:
    void cmd_set_position(char * pStr, bool use_checksum);
    void cmd_set_position_wl(char * pStr, bool use_checksum);
    void cmd_set_velocity(char * pStr, bool use_checksum);
    void cmd_set_torque(char * pStr, bool use_checksum);
    void cmd_set_trapezoid_trajectory(char * pStr, bool use_checksum);
    void cmd_get_feedback(char * pStr, bool use_checksum);
    void cmd_help(char * pStr, bool use_checksum);
    void cmd_info_dump(char * pStr, bool use_checksum);
    void cmd_system_ctrl(char * pStr, bool use_checksum);
    void cmd_read_property(char * pStr, bool use_checksum);
    void cmd_write_property(char * pStr, bool use_checksum);
    void cmd_update_axis_wdg(char * pStr, bool use_checksum);
    void cmd_unknown(char * pStr, bool use_checksum);
    void cmd_encoder(char * pStr, bool use_checksum);

    template<typename ... TArgs> void respond(bool include_checksum, const char * fmt, TArgs&& ... args);
    void process_line(char *  buffer);
    void on_read_finished(bool result);

    uint8_t* rx_end_ = nullptr; // non-zero if an RX operation has finished but wasn't handled yet because the TX channel was busy

    uint8_t rx_buf_[MAX_LINE_LENGTH];
    bool read_active_ = true;
};

#endif // __ASCII_PROTOCOL_HPP
