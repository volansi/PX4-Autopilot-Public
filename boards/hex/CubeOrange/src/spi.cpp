/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

/** ArduPilot CubeOrange SPI Device Table
 *    PC1 MAG_CS CS
 *    PC2 MPU_CS CS
 *    PC13 GYRO_EXT_CS CS
 *    PC14 BARO_EXT_CS CS
 *    PC15 ACCEL_EXT_CS CS
 *    PD7 BARO_CS CS
 *    PE4 MPU_EXT_CS CS
 *    PD10 FRAM_CS CS SPEED_VERYLOW
 *
 *    SPIDEV ms5611         SPI1 DEVID3  BARO_CS      MODE3 20*MHZ 20*MHZ
 *    SPIDEV ms5611_ext     SPI4 DEVID2  BARO_EXT_CS  MODE3 20*MHZ 20*MHZ
 *    SPIDEV mpu6000        SPI1 DEVID4  MPU_CS       MODE3  2*MHZ  8*MHZ
 *    SPIDEV icm20608-am    SPI1 DEVID2  ACCEL_EXT_CS MODE3  4*MHZ  8*MHZ
 *    SPIDEV mpu9250        SPI1 DEVID4  MPU_CS       MODE3  4*MHZ  8*MHZ
 *    SPIDEV mpu9250_ext    SPI4 DEVID1  MPU_EXT_CS   MODE3  4*MHZ  8*MHZ
 *    SPIDEV icm20948       SPI1 DEVID4  MPU_CS       MODE3  4*MHZ  8*MHZ
 *    SPIDEV icm20948_ext   SPI4 DEVID1  MPU_EXT_CS   MODE3  4*MHZ  8*MHZ
 *    SPIDEV hmc5843        SPI1 DEVID5  MAG_CS       MODE3 11*MHZ 11*MHZ
 *    SPIDEV lsm9ds0_g      SPI1 DEVID1  GYRO_EXT_CS  MODE3 11*MHZ 11*MHZ
 *    SPIDEV lsm9ds0_am     SPI1 DEVID2  ACCEL_EXT_CS MODE3 11*MHZ 11*MHZ
 *    SPIDEV lsm9ds0_ext_g  SPI4 DEVID4  GYRO_EXT_CS  MODE3 11*MHZ 11*MHZ
 *    SPIDEV lsm9ds0_ext_am SPI4 DEVID3  ACCEL_EXT_CS MODE3 11*MHZ 11*MHZ
 *    SPIDEV icm20602_ext   SPI4 DEVID4  GYRO_EXT_CS  MODE3  4*MHZ  8*MHZ
 *    SPIDEV ramtron        SPI2 DEVID10 FRAM_CS      MODE3  8*MHZ  8*MHZ
 *    SPIDEV external0m0    SPI4 DEVID5  MPU_EXT_CS   MODE0  2*MHZ  2*MHZ
 *    SPIDEV external0m1    SPI4 DEVID5  MPU_EXT_CS   MODE1  2*MHZ  2*MHZ
 *    SPIDEV external0m2    SPI4 DEVID5  MPU_EXT_CS   MODE2  2*MHZ  2*MHZ
 *    SPIDEV external0m3    SPI4 DEVID5  MPU_EXT_CS   MODE3  2*MHZ  2*MHZ
 *    SPIDEV pixartPC15     SPI4 DEVID13 ACCEL_EXT_CS MODE3  2*MHZ  2*MHZ
 *
 *    # three IMUs, but allow for different varients. First two IMUs are
 *    # isolated, 3rd isn't
 *    IMU Invensense SPI:mpu9250_ext ROTATION_PITCH_180
 *
 *    # the 3 rotations for the LSM9DS0 driver are for the accel, the gyro
 *    # and the H varient of the gyro
 *    IMU LSM9DS0 SPI:lsm9ds0_ext_g SPI:lsm9ds0_ext_am ROTATION_ROLL_180_YAW_270 ROTATION_ROLL_180_YAW_90 ROTATION_ROLL_180_YAW_90
 *
 *    # 3rd non-isolated IMU
 *    IMU Invensense SPI:mpu9250 ROTATION_YAW_270
 *
 *    # alternative IMU set for newer cubes
 *    IMU Invensense SPI:icm20602_ext ROTATION_ROLL_180_YAW_270
 *    IMU Invensensev2 SPI:icm20948_ext ROTATION_PITCH_180
 *    IMU Invensensev2 SPI:icm20948 ROTATION_YAW_270
 *
 *    define HAL_DEFAULT_INS_FAST_SAMPLE 7
 *
 *    # two baros
 *    BARO MS56XX SPI:ms5611_ext
 *    BARO MS56XX SPI:ms5611
 *
 *    # two compasses. First is in the LSM303D
 *    COMPASS LSM303D SPI:lsm9ds0_ext_am ROTATION_YAW_270
 *    # 2nd compass is part of the 2nd invensense IMU
 *    COMPASS AK8963:probe_mpu9250 1 ROTATION_YAW_270
 *
 *    # compass as part of ICM20948 on newer cubes
 *    COMPASS AK09916:probe_ICM20948 0 ROTATION_ROLL_180_YAW_90
 *
 *  Final SPI Table:
 *
 *  BUS     TYPE       DEVICE     CS     ROTATION
 *  ----    ----       -------    ---    -----------------
 *  SPI1    IMU        MPU9250    E4     PITCH-180
 *  SPI4    GYRO       LSM9DS0    C13    ROLL-180-YAW-270
 *  SPI4    ACC        LSM9DS0    C15    ROLL-180-YAW-90
 *  SPI4    IMU/MAG    MPU9250    C2     YAW-270
 *  SPI4    IMU        ICM20602   C13    ROLL-180-YAW-270
 *  SPI4    IMU        ICM20948   E4     PITCH-180
 *  SPI1    IMU/MAG    ICM20948   C2     YAW-270 (IMU) / ROLL-180-YAW-90 (MAG)
 *  SPI4    BARO       MS5611     C14
 *  SPI1    BARO       MS5611     D7
 *  SPI4    MAG        LSM303D    C15    YAW-270 (MAG)
 */


constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortE, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20948, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
	}, {GPIO::PortE, GPIO::Pin3}
		  ),

	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
	}),

	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM9DS1_AG, SPI::CS{GPIO::PortC, GPIO::Pin13}),
		initSPIDevice(DRV_MAG_DEVTYPE_ST_LSM9DS1_M, SPI::CS{GPIO::PortC, GPIO::Pin15}),
		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, SPI::CS{GPIO::PortC, GPIO::Pin13}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortC, GPIO::Pin14}),
	}),
};

/* TODO: Figure out which of these are actually relevant (or see fmu-v3 for version-based config)
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM9DS1_AG, SPI::CS{GPIO::PortC, GPIO::Pin13}),
		initSPIDevice(DRV_MAG_DEVTYPE_ST_LSM9DS1_M, SPI::CS{GPIO::PortC, GPIO::Pin15}),
		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, SPI::CS{GPIO::PortC, GPIO::Pin13}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20948, SPI::CS{GPIO::PortE, GPIO::Pin4}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortC, GPIO::Pin14}),
	}),
*/

// initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
