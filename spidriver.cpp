/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2022 Bernd Ocklin <bernd@ocklin.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "spidriver.h"

/**
 * \brief Initialise the SPI port's GPIO lines
 */
void SpiDriver::InitGPIOs() {

    // GPIO59 is the SPISOMIA.
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    // GPIO58 is the SPISIMOA clock pin.
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(58, GPIO_QUAL_ASYNC);

    // GPIO61 is the SPISTEA.
    GPIO_setMasterCore(61, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_61_SPISTEA);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(61, GPIO_QUAL_ASYNC);

    // GPIO160 is the SPICLKA.
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);
}

/**
 * \brief Initialise the SPI port and associated clocks
 */
void SpiDriver::InitSPIPort()
{
    // TODO

    // Assert the SPI ~CS enable line before we turn off the
    // pull up to avoid glitches

    SPI_disableModule(SPIA_BASE);

    // Tesla run the device at 2.4MHz so we also run slower than rated 5MHz
    // to be on the safe side.
    // The TLF35584 requires CPOL = 0, CPHA = 0 and 16-bit MSB transfers with software
    // controlled chip-select control around each transfer.
    // MASTER mode, unidirectional full duplex

    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 500000, 16);

    SPI_disableFIFO(SPIA_BASE);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);


    SPI_enableModule(SPIA_BASE);
}

void SpiDriver::WriteData(uint16_t data)
{
    SPI_writeDataNonBlocking(SPIA_BASE, data);
}

uint16_t SpiDriver::ReadData()
{
    return SPI_readDataBlockingNonFIFO(SPIA_BASE);
}
