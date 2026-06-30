#include"IMU.h"
#include"BMI270Config.h"

#define I2C_NUM I2C_NUM_0

#define SCL_PIN GPIO_NUM_21
#define SDA_PIN GPIO_NUM_22

/*===============后期可改参数================*/
static const uint8_t REG_ACC_RANGE_DATA  = 0x02;        /*0x00 +/-2g; 0x01 +/-4g; 0x02 +/-8g ;0x03 +/-16g*/
uint8_t acc_conf = (1 << 7) | (2 << 4) | 9;             /* ACC_CONF: 性能模式(bit7=1), 普通滤波(bit6-4=2), ODR=200Hz(bit3-0=9) */

static const uint8_t REG_GYR_RANGE_DATA  = 0x00;        /*0x00 2000 ; 0x01 1000 ; 0x02 500 ; 0x03 250 ; 0x04 125 */
uint8_t gyr_conf = (1 << 7) | (0 << 6) | (2 << 4) | 9;  /* GYR_CONF: 性能模式(bit7=1), 普通噪声(bit6=0), 普通滤波(bit5-4=2), ODR=200Hz(bit3-0=9) */

uint8_t REG_PWR_CONF_DATA=0x00;                         /*关闭高级省电，关闭 FIFO 自唤醒功能，关闭快速上电*/

static float bmi270_gyroScale=0.0174533f /16.4;         // 角速度量程缩放因子
static float bmi270_accelScale=1.f/4096;                // 加速度计量程缩放因子(16g/65536)

/*===============不可改参数================*/
static const uint8_t BMI270_I2C_ADDR = 0x69;
static const uint8_t REG_CHIP_ID         = 0x00;
static const uint8_t REG_PWR_CONF        = 0x7C;//电源
static const uint8_t REG_PWR_CTRL        = 0x7D;
static const uint8_t REG_ACC_CONF        = 0x40;
static const uint8_t REG_ACC_RANGE       = 0x41;//加速度范围地址
static const uint8_t REG_GYR_CONF        = 0x42;
static const uint8_t REG_GYR_RANGE       = 0x43;
static const uint8_t REG_INIT_CTRL       = 0x59;
static const uint8_t REG_INIT_DATA       = 0x5E;
static const uint8_t REG_INTERNAL_STATUS = 0x21;
static const uint8_t REG_ACC_X_LSB       = 0x0C;   /* DATA_8 */
static const uint8_t REG_GYR_X_LSB       = 0x12;   /* DATA_14 */

// 添加全局变量保存零偏
int16_t gyro_offset_x = 0;
int16_t gyro_offset_y = 0;
int16_t gyro_offset_z = 0;
// 初始倾斜偏移
static float roll_offset  = 0.0f;
static float pitch_offset = 0.0f;

static uint8_t imu_cali_finish = 0;
#define ATT_CALI_SAMPLE_CNT  50

static esp_err_t i2c_write_byte(uint8_t reg, uint8_t data){
    uint8_t buf[2]={reg,data};
    return i2c_master_write_to_device(I2C_NUM,BMI270_I2C_ADDR,buf,2,pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_byte(uint8_t reg, uint8_t *data){
    return i2c_master_write_read_device(I2C_NUM,BMI270_I2C_ADDR,&reg,1,data,1,pdMS_TO_TICKS(100));
}

static esp_err_t i2c_write_bytes(uint8_t reg, uint8_t *data,size_t len){
    uint8_t *buf=malloc(len+1);
    if(buf==NULL)return ESP_ERR_NO_MEM;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM, BMI270_I2C_ADDR, buf, len + 1, pdMS_TO_TICKS(500));//一次性写入
    free(buf);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *data,size_t len){
    return i2c_master_write_read_device(I2C_NUM,BMI270_I2C_ADDR,&reg,1,data,len,pdMS_TO_TICKS(100));
}

esp_err_t BMI270_Init(void){
    i2c_config_t BMI270_InitStructure={
        .master.clk_speed=400000,
        .mode=I2C_MODE_MASTER,
        .scl_io_num=SCL_PIN,
        .sda_io_num=SDA_PIN,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
    };
    esp_err_t ret=i2c_param_config(I2C_NUM,&BMI270_InitStructure);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_NUM,BMI270_InitStructure.mode,0,0,0);
}

esp_err_t bmi270_check_chip_id(void)
{
    uint8_t id;
    esp_err_t ret = i2c_read_byte(REG_CHIP_ID, &id);
    
    if (ret != ESP_OK) {
        printf("I2C read REG_CHIP_ID failed! Error code: 0x%x (Is address correct? Check wiring/pull-ups)\n", ret);
        return ret;
    };
    return (id == 0x24) ? ESP_OK : ESP_FAIL;
}

esp_err_t bmi270_init_device(IMU_DATA *_this){
     uint8_t status;
    int timeout = 0;

    i2c_write_byte(REG_PWR_CONF,REG_PWR_CONF_DATA);

    esp_rom_delay_us(500);//等待上电
    
    i2c_write_byte(REG_INIT_CTRL, 0x00);
    for (int i = 0; i < 256; i++) {
        // 设定当前的字写入地址
        i2c_write_byte(0x5B, 0x00); // INIT_ADDR_0 (字地址低4位，始终为0)
        i2c_write_byte(0x5C, i);    // INIT_ADDR_1 (字地址高8位，等于 i)
        
        // 写入 32 字节的配置段
        i2c_write_bytes(REG_INIT_DATA, &bmi270_config_file[i * 32], 32);

        esp_rom_delay_us(20); // 每次微小延时，保证芯片稳定接收
    }
    i2c_write_byte(REG_INIT_CTRL, 0x01);

    while (timeout < 30) {
        i2c_read_byte(REG_INTERNAL_STATUS, &status);
        if ((status & 0x0F) == 0x01) break;//等于0x01初始化完成
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout++;
    }
    if (timeout >= 30) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t BMI270_Acc_config(void){
    i2c_write_byte(REG_ACC_CONF,acc_conf);
    return i2c_write_byte(REG_ACC_RANGE, REG_ACC_RANGE_DATA);
}

esp_err_t BMI270_Gyr_config(void){
    i2c_write_byte(REG_GYR_CONF,gyr_conf);
    return i2c_write_byte(REG_GYR_RANGE,REG_GYR_RANGE_DATA);
}

esp_err_t bmi270_enable_sensors(void)
{
    return i2c_write_byte(REG_PWR_CTRL, (1 << 2) | (1 << 1));/* 使能加速度计(bit2)和陀螺仪(bit1) */
}

esp_err_t bmi270_read_data(IMU_DATA *_this)
{
    uint8_t data[6];
    i2c_read_bytes(REG_GYR_X_LSB, data, 6);

    _this->gyro_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    _this->gyro_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    _this->gyro_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    i2c_read_bytes(REG_ACC_X_LSB, data, 6);

    _this->acc_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    _this->acc_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    _this->acc_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return ESP_OK;
}

esp_err_t bmi270_full_init(IMU_DATA *_this)
{
    esp_err_t ret;
    _this->q0 = 1.0f;
	_this->q1 = 0.0f;
	_this->q2 = 0.0f;
	_this->q3 = 0.0f;
    ret = BMI270_Init();
    if (ret != ESP_OK) {
        printf("I2C driver installation failed! Error: 0x%x\n", ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = bmi270_check_chip_id();
    if (ret != ESP_OK) return ret;

    ret = bmi270_init_device(_this);
    if (ret != ESP_OK) return ret;

    ret = BMI270_Acc_config();
    if (ret != ESP_OK) {
        printf("Accelerometer configuration failed!\n");
        return ret;
    }

    ret = BMI270_Gyr_config();
    if (ret != ESP_OK) {
        printf("Gyroscope configuration failed!\n");
        return ret;
    }

    ret = bmi270_enable_sensors();
    if (ret != ESP_OK) {
        printf("Failed to enable sensors!\n");
        return ret;
    }
    return ret;
}

const float bmi270_dt = 0.005f;
#define ICM20602_INTEGRAL_LIMIT  0.5f
// 快速平方根倒数算法
static inline float invSqrt(float x) {
    union {
        float f;
        long i;
    } u;
    float halfx = 0.5f * x;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    u.f = u.f * (1.5f - (halfx * u.f * u.f));  // 一次牛顿迭代
    return u.f;
}
static float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;
    else return value;
}
void BMI270_Attitude_Calibrate(IMU_DATA *_this)
{
    if(imu_cali_finish != 0) return;

    float sum_roll = 0.0f, sum_pitch = 0.0f;

    for(int i = 0; i < ATT_CALI_SAMPLE_CNT; i++)
    {
        BMI270_Get_Angle_Plus(_this);
        sum_roll  += _this->roll;
        sum_pitch += _this->pitch;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    roll_offset  = sum_roll / ATT_CALI_SAMPLE_CNT;
    pitch_offset = sum_pitch / ATT_CALI_SAMPLE_CNT;
    imu_cali_finish = 1;

}
// void BMI270_Attitude_Calibrate(IMU_DATA *_this)
// {
//     if(imu_cali_finish != 0) return;

//     roll_offset  = 1.2f;   // 改成你自己的数值
//     pitch_offset = -0.7f;

//     imu_cali_finish = 1;
// }
void BMI270_Get_Angle_Plus(IMU_DATA* _this) {

    static float Kp, Ki;
    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;

    // 读取原始数据
    bmi270_read_data(_this);
     
    // 转换为物理量
    float ax = (float)_this->acc_x * bmi270_accelScale;
    float ay = (float)_this->acc_y * bmi270_accelScale;
    float az = (float)_this->acc_z * bmi270_accelScale;
    float gx = (float)(_this->gyro_x - gyro_offset_x) * bmi270_gyroScale;
    float gy = (float)(_this->gyro_y - gyro_offset_y) * bmi270_gyroScale;
    float gz = (float)(_this->gyro_z - gyro_offset_z) * bmi270_gyroScale;

	// 加速度幅值（用于自适应增益）
	float accMag = ax * ax + ay * ay + az * az;
    //值越大越相信加速度计
	Kp = 1.2f;
	Ki = 0.01f;
//	Kp = 0.1f;
//	Ki = 0.00001f;
	// 加速度计有效时校正姿态
	if (accMag > 0.01f) {
        float recipNorm = invSqrt(accMag);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 计算重力方向误差
        float vx = 2.0f * (_this->q1 * _this->q3 - _this->q0 * _this->q2);
        float vy = 2.0f * (_this->q0 * _this->q1 + _this->q2 * _this->q3);
        float vz = _this->q0 * _this->q0 - _this->q1 * _this->q1 - _this->q2 * _this->q2 + _this->q3 * _this->q3;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // 积分项校正
        if (Ki > 0.0f) {
            integralX += ex * bmi270_dt;
            integralY += ey * bmi270_dt;
            integralZ += ez * bmi270_dt;
            //积分限幅
            integralX = constrain(integralX, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);
            integralY = constrain(integralY, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);
            integralZ = constrain(integralZ, -ICM20602_INTEGRAL_LIMIT, ICM20602_INTEGRAL_LIMIT);

            gx += Ki * integralX;
            gy += Ki * integralY;
            gz += Ki * integralZ;
        }

        // 比例项校正
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // 四元数更新（核心计算）
    float qDot0 = 0.5f * (-_this->q1 * gx - _this->q2 * gy - _this->q3 * gz);
    float qDot1 = 0.5f * (_this->q0 * gx + _this->q2 * gz - _this->q3 * gy);
    float qDot2 = 0.5f * (_this->q0 * gy - _this->q1 * gz + _this->q3 * gx);
    float qDot3 = 0.5f * (_this->q0 * gz + _this->q1 * gy - _this->q2 * gx);

    _this->q0 += qDot0 * bmi270_dt;
    _this->q1 += qDot1 * bmi270_dt;
    _this->q2 += qDot2 * bmi270_dt;
    _this->q3 += qDot3 * bmi270_dt;

    // 四元数归一化
    float norm = invSqrt(_this->q0 * _this->q0 + _this->q1 * _this->q1 + _this->q2 * _this->q2 + _this->q3 * _this->q3);
    _this->q0 *= norm;
    _this->q1 *= norm;
    _this->q2 *= norm;
    _this->q3 *= norm;

    // 计算欧拉角（四元数转欧拉角核心）
    _this->roll        = atan2f(2.0f * (_this->q0 * _this->q1 + _this->q2 * _this->q3), 1.0f - 2.0f * (_this->q1 * _this->q1 + _this->q2 * _this->q2)) * 57.29578f;
    _this->pitch       = asinf(2.0f * (_this->q0 * _this->q2 - _this->q3 * _this->q1)) * 57.29578f;
    float current_yaw = atan2f(2.0f * (_this->q0 * _this->q3 + _this->q1 * _this->q2), 1.0f - 2.0f * (_this->q2 * _this->q2 + _this->q3 * _this->q3)) * 57.29578f;
	// 减去初始倾斜偏移
    _this->roll  -= roll_offset;
    _this->pitch -= pitch_offset;

    //_this->yaw = current_yaw;
    //角度无限叠加(如果不需要则注释下面这里然后打开上面这个注释,这样就是0~180)
   static float unwrapped_yaw = 0.0f;
   static float last_yaw = 0.0f;
   static uint8_t first_run   = 1;
   if (first_run) {
       unwrapped_yaw = current_yaw;
       last_yaw = current_yaw;
       first_run     = 0;
   } else {
       float diff = current_yaw - last_yaw;

       if (diff > 180.0f) {
           diff -= 360.0f;
       } else if (diff < -180.0f) {
           diff += 360.0f;
       }
       unwrapped_yaw += diff;
       last_yaw = current_yaw;
   }
   _this->yaw = unwrapped_yaw; // 输出连续yaw角
}


void BMI270_Calibrate(IMU_DATA *_this) {
    long sum_x = 0, sum_y = 0, sum_z = 0;
    printf("Calibrating Gyro, please DO NOT move...\n");
    for (int i = 0; i < 500; i++) {
        bmi270_read_data(_this);
        sum_x += _this->gyro_x;
        sum_y += _this->gyro_y;
        sum_z += _this->gyro_z;
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
    gyro_offset_x = sum_x / 500;
    gyro_offset_y = sum_y / 500;
    gyro_offset_z = sum_z / 500;
    printf("Gyro Offsets: X:%d, Y:%d, Z:%d\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}