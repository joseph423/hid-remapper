#ifndef TEST_FAKE_I2C_H_
#define TEST_FAKE_I2C_H_

#include <cstdint>
#include <deque>
#include <vector>

constexpr uint32_t I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS = 0x40;
constexpr uint32_t I2C_IC_RAW_INTR_STAT_RX_OVER_BITS = 0x02;
constexpr uint32_t I2C_IC_RAW_INTR_STAT_STOP_DET_BITS = 0x200;
constexpr uint32_t I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS = 1;
constexpr uint32_t I2C_IC_DATA_CMD_CMD_BITS = 0x100;
constexpr uint32_t I2C_IC_DATA_CMD_STOP_BITS = 0x200;
constexpr uint32_t I2C_IC_DATA_CMD_RESTART_BITS = 0x400;

struct i2c_hw_t;
// Models FIFO reads/writes and clear-on-read registers, not bus timing.
struct FakeRegister {
    enum Kind { Data,
        RxLevel,
        TxLevel,
        ClearStop,
        ClearAbort } kind;
    i2c_hw_t* hw;
    operator uint32_t() const;
    void operator=(uint32_t value);
};

struct i2c_hw_t {
    uint32_t con = 0, fs_scl_hcnt = 0, fs_scl_lcnt = 0, sda_hold = 0, fs_spklen = 0;
    uint32_t enable = 0, tar = 0, tx_tl = 0, rx_tl = 0;
    uint32_t raw_intr_stat = 0, tx_abrt_source = 0;
    FakeRegister data_cmd{ FakeRegister::Data, this };
    FakeRegister rxflr{ FakeRegister::RxLevel, this };
    FakeRegister txflr{ FakeRegister::TxLevel, this };
    FakeRegister clr_stop_det{ FakeRegister::ClearStop, this };
    FakeRegister clr_tx_abrt{ FakeRegister::ClearAbort, this };
    std::deque<uint32_t> tx, rx;
    std::vector<uint32_t> written;
};

struct i2c_inst_t {
    i2c_hw_t* hw;
};
extern i2c_inst_t *i2c0, *i2c1;
void i2c_init(i2c_inst_t*, uint32_t);
#endif
