/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "RF24.h"
#include <Digital.h>
#include <BitManipulations.h>
#include <Data/CStringArray.h>
#include <Clock.h>
#include <Platform/Timers.h>

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
	packet.prepare();
	packet.out.set8(R_REGISTER | (REGISTER_MASK & reg));
	assert(len < sizeof(inbuf));
	memset(inbuf, 0, sizeof(inbuf));
	packet.in.set(inbuf, 1 + len);
	spidev.execute(packet);
	if(len != 0) {
		memcpy(buf, &inbuf[1], len);
	}
	return inbuf[0];
}

uint8_t RF24::read_register(uint8_t reg)
{
	packet.prepare();
	packet.out.set8(R_REGISTER | (REGISTER_MASK & reg));
	packet.in.set16(0);
	spidev.execute(packet);
	return packet.in.data[1];
}

uint8_t RF24::write_register(uint8_t reg, const void* buf, uint8_t len)
{
	packet.prepare();
	outbuf[0] = W_REGISTER | (REGISTER_MASK & reg);
	assert(len < sizeof(outbuf));
	memcpy(&outbuf[1], buf, len);
	packet.out.set(outbuf, 1 + len);
	packet.in.set8(0);
	spidev.execute(packet);
	return packet.in.data8;
}

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
	debug_i("write_register(%02x,%02x)", reg, value);

	packet.prepare();
	packet.out.set16((value << 8) | W_REGISTER | (REGISTER_MASK & reg));
	packet.in.set8(0);
	spidev.execute(packet);
	return packet.in.data8;
}

uint8_t RF24::write_payload(const void* buf, uint8_t data_len, uint8_t writeType)
{
	data_len = std::min(data_len, payload_size);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	debug_i("[Writing %u bytes %u blanks]", data_len, blank_len);

	packet.prepare();
	outbuf[0] = writeType;
	memcpy(&outbuf[1], buf, data_len);
	memset(&outbuf[1 + data_len], 0, blank_len);
	packet.out.set(outbuf, 1 + data_len + blank_len);
	packet.in.set8(0);
	spidev.execute(packet);
	return packet.in.data8;
}

uint8_t RF24::read_payload(void* buf, uint8_t data_len)
{
	data_len = std::min(data_len, payload_size);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	debug_i("[Reading %u bytes %u blanks]", data_len, blank_len);

	packet.prepare();
	packet.out.set8(R_RX_PAYLOAD);
	memset(inbuf, 0, sizeof(inbuf));
	packet.in.set(inbuf, 1 + data_len + blank_len);
	spidev.execute(packet);
	memcpy(buf, &inbuf[1], data_len);
	return packet.in.data8;
}

uint8_t RF24::flush_rx()
{
	return spiTrans(FLUSH_RX);
}

uint8_t RF24::flush_tx()
{
	return spiTrans(FLUSH_TX);
}

uint8_t RF24::spiTrans(uint8_t cmd)
{
	packet.prepare();
	packet.out.set8(cmd);
	packet.in.set8(0);
	spidev.execute(packet);
	return packet.in.data8;
}

uint8_t RF24::get_status()
{
	return spiTrans(RF24_NOP);
}

#ifndef RF24_MINIMAL

void RF24::print_status(uint8_t status)
{
	m_printf(_F("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"), status,
			 (status & _BV(RX_DR)) ? 1 : 0, (status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0,
			 ((status >> RX_P_NO) & 0x07), (status & _BV(TX_FULL)) ? 1 : 0);
}

void RF24::print_observe_tx(uint8_t value)
{
	m_printf(_F("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"), value, (value >> PLOS_CNT) & 0x0F,
			 (value >> ARC_CNT) & 0x0F);
}

void RF24::print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
	m_printf(_F("%s\t ="), name);
	while(qty--) {
		m_printf(_F(" 0x%02x"), read_register(reg++));
	}
	m_puts("\r\n");
}

void RF24::print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
	m_printf(_F("%s \t ="), name);

	while(qty--) {
		uint8_t buffer[addr_width];
		read_register(reg++, buffer, addr_width);

		m_puts(_F(" 0x"));
		auto bufptr = buffer + addr_width;
		while(--bufptr >= buffer) {
			m_printf(_F("%02x"), *bufptr);
		}
	}

	m_puts("\r\n");
}

void RF24::printDetails()
{
	m_printf(_F("SPI Speed\t = %u MHz\n"), spidev.getSpeed() / 1000000U);

	print_status(get_status());

	print_address_register(_F("RX_ADDR_P0-1"), RX_ADDR_P0, 2);
	print_byte_register(_F("RX_ADDR_P2-5"), RX_ADDR_P2, 4);
	print_address_register(_F("TX_ADDR\t"), TX_ADDR);

	print_byte_register(_F("RX_PW_P0-6"), RX_PW_P0, 6);
	print_byte_register(_F("EN_AA\t"), EN_AA);
	print_byte_register(_F("EN_RXADDR"), EN_RXADDR);
	print_byte_register(_F("RF_CH\t"), RF_CH);
	print_byte_register(_F("RF_SETUP"), RF_SETUP);
	print_byte_register(_F("CONFIG\t"), NRF_CONFIG);
	print_byte_register(_F("DYNPD/FEATURE"), DYNPD, 2);

	CStringArray dataRateStrings(F("1MBPS\0"
								   "2MBPS\0"
								   "250KBPS\0"));
	m_printf(_F("Data Rate\t = %s\r\n"), dataRateStrings[getDataRate()]);

	m_printf(_F("Model\t\t = nRF24L01%c\r\n"), isPVariant() ? '+' : ' ');

	CStringArray crcLengthStrings(F("Disabled\0"
									"8 bits\0"
									"16 bits\0"));
	m_printf(_F("CRC Length\t = %s\r\n"), crcLengthStrings[getCRCLength()]);

	CStringArray powerStrings(F("MIN\0"
								"LOW\0"
								"HIGH\0"
								"MAX\0"));
	m_printf(_F("PA Power\t = PA_%s\r\n"), powerStrings[getPALevel()]);
}

#endif // RF24_MINIMAL

bool RF24::begin(int8_t cepin, SpiMaster* spi, uint32_t spiSpeed)
{
	ce_pin = cepin;

	pipe0_reading_address[0] = 0;

	// Initialize pins
	if(ce_pin >= 0) {
		pinMode(ce_pin, OUTPUT);
		ce(LOW);
	}
	spidev.begin(spi);
	spidev.setBitOrder(MSBFIRST);
	spidev.setMode(SPI_MODE0);
	spidev.setSpeed(spiSpeed);
	packet.duplex = true;

	debug_i("spiSpeed = %u, set = %u, clockReg = 0x%08x", spiSpeed, spidev.getSpeed(), spidev.getClockReg());

	// Must allow the radio time to settle else configuration bits will not necessarily stick.
	// This is actually only required following power up but some settling time also appears to
	// be required after resets too. For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	delay(5);

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	setRetries(5, 15);

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	setDataRate(RF24_1MBPS);

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	toggle_features();
	write_register(FEATURE, 0);
	write_register(DYNPD, 0);
	dynamic_payloads_enabled = false;
	ack_payloads_enabled = false;

	// Reset current status
	// Notice reset and flush is the last thing we do
	write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	setChannel(76);

	// Flush buffers
	flush_rx();
	flush_tx();

	// Clear CONFIG register, Enable PTX, Power Up & 16-bit CRC
	// Do not write CE high so radio will remain in standby I mode
	// PTX should use only 22uA of power
	write_register(NRF_CONFIG, _BV(EN_CRC) | _BV(CRCO));
	config_reg = read_register(NRF_CONFIG);

	powerUp();

	// if config is not set correctly then there was a bad response from module
	return config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP));
}

bool RF24::isChipConnected()
{
	uint8_t setup = read_register(SETUP_AW);
	return (setup >= 1) && (setup <= 3);
}

void RF24::startListening()
{
#if !defined(RF24_TINY) && !defined(LITTLEWIRE)
	powerUp();
#endif
	/* Notes Once ready for next release
     * 1. Can update stopListening() to use config_reg var and ack_payloads_enabled var instead of SPI rx/tx
     * 2. Update txDelay defaults: 240 for 2MBPS, 280 for 1MBPS, 505 for 250KBPS per initial testing
     * 3. Allows time for slower devices to update with the faster startListening() function prior to updating stopListening() & adjusting txDelay
     */
	config_reg |= _BV(PRIM_RX);
	write_register(NRF_CONFIG, config_reg);
	write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	ce(HIGH);
	// Restore the pipe0 adddress, if exists
	if(pipe0_reading_address[0] > 0) {
		write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
	} else {
		closeReadingPipe(0);
	}

	if(ack_payloads_enabled) {
		flush_tx();
	}
}

static uint8_t child_pipe_enable(uint8_t child)
{
	return ERX_P0 + child;
}

void RF24::stopListening()
{
	ce(LOW);

	delayMicroseconds(txDelay);

	if(read_register(FEATURE) & _BV(EN_ACK_PAY)) {
		delayMicroseconds(txDelay);
		flush_tx();
	}
	config_reg &= ~_BV(PRIM_RX);
	write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~_BV(PRIM_RX));

	// Enable RX on pipe0
	write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(child_pipe_enable(0)));

	// delayMicroseconds(100);
}

void RF24::powerDown()
{
	ce(LOW); // Guarantee CE is low on powerDown
	config_reg &= ~_BV(PWR_UP);
	write_register(NRF_CONFIG, config_reg);
}

// Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24::powerUp()
{
	// if not powered up then power up and wait for the radio to initialize
	if(!(config_reg & _BV(PWR_UP))) {
		config_reg |= _BV(PWR_UP);
		write_register(NRF_CONFIG, config_reg);

		// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
		// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
		// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
		delay(5);
	}
}

#if defined(RF24_FAILURE_HANDLING)

void RF24::errNotify()
{
	debug_e("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.");
#if defined(RF24_FAILURE_HANDLING)
	failureFlag = true;
#else
	delay(5000);
#endif
}

#endif

void RF24::setChannel(uint8_t channel)
{
	const uint8_t max_channel = 125;
	write_register(RF_CH, std::min(channel, max_channel));
}

uint8_t RF24::getChannel()
{
	return read_register(RF_CH);
}

// Similar to the previous write, clears the interrupt flags
bool RF24::write(const void* buf, uint8_t len, bool multicast)
{
	// Start Writing
	startFastWrite(buf, len, multicast);

// Wait until complete or failed
#if defined(RF24_FAILURE_HANDLING)
	OneShotFastMs timer(TIMEOUT_MS);
#endif

	while(!(get_status() & (_BV(TX_DS) | _BV(MAX_RT)))) {
#if defined(RF24_FAILURE_HANDLING)
		if(timer.expired()) {
			errNotify();
			return false;
		}
#endif
	}

	ce(LOW);

	uint8_t status = write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Max retries exceeded
	if(status & _BV(MAX_RT)) {
		flush_tx(); // Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return false;
	}

	return true;
}

// For general use, the interrupt flags are not important to clear
bool RF24::writeBlocking(const void* buf, uint8_t len, uint32_t timeout)
{
	// Block until the FIFO is NOT full.
	// Keep track of the MAX retries and set auto-retry if seeing failures
	// This way the FIFO will fill up and allow blocking until packets go through
	// The radio will auto-clear everything in the FIFO as long as CE remains high

	OneShotFastMs timer(timeout);

	// Blocking only if FIFO is full. This will loop and block until TX is successful or timeout
	while((get_status() & (_BV(TX_FULL)))) {
		if(get_status() & _BV(MAX_RT)) {
			// MAX Retries have been reached: set re-transmit and clear the MAX_RT interrupt flag
			reUseTX();
			if(timer.expired()) {
				return false;
			}
		}
#if defined(RF24_FAILURE_HANDLING)
		if(timer.elapsedTicks() > timer.checkTime<TIMEOUT_MS>()) {
			errNotify();
			return false;
		}
#endif
	}

	// Start Writing
	startFastWrite(buf, len, false); // Write the payload if a buffer is clear

	return true;
}

void RF24::reUseTX()
{
	write_register(NRF_STATUS, _BV(MAX_RT)); // Clear max retry flag
	spiTrans(REUSE_TX_PL);
	ce(LOW); // Re-Transfer packet
	ce(HIGH);
}

bool RF24::writeFast(const void* buf, uint8_t len, bool multicast)
{
	// Block until the FIFO is NOT full.
	// Keep track of the MAX retries and set auto-retry if seeing failures
	// Return 0 so the user can control the retrys and set a timer or failure counter if required
	// The radio will auto-clear everything in the FIFO as long as CE remains high

#if defined(RF24_FAILURE_HANDLING)
	OneShotFastMs timer(TIMEOUT_MS);
#endif

	// Blocking only if FIFO is full. This will loop and block until TX is successful or fail
	while((get_status() & (_BV(TX_FULL)))) {
		if(get_status() & _BV(MAX_RT)) {
			// The previous payload has been retransmitted
			// From the user perspective, if you get false just keep trying to send the same payload
			return false;
		}
#if defined(RF24_FAILURE_HANDLING)
		if(timer.expired()) {
			errNotify();
			return false;
		}
#endif
	}
	// Start Writing
	startFastWrite(buf, len, multicast);

	return true;
}

bool RF24::writeFast(const void* buf, uint8_t len)
{
	return writeFast(buf, len, false);
}

// Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
// In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
// Otherwise we enter Standby-II mode, which is still faster than standby mode
// Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void RF24::startFastWrite(const void* buf, uint8_t len, bool multicast, bool startTx)
{ // TMRh20

	write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	if(startTx) {
		ce(HIGH);
	}
}

// Added the original startWrite back in so users can still use interrupts, ack payloads, etc
// Allows the library to pass all tests
void RF24::startWrite(const void* buf, uint8_t len, bool multicast)
{
	write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	if(ce_pin >= 0) {
		ce(HIGH);
#if !defined(F_CPU) || F_CPU > 20000000
		delayMicroseconds(10);
#endif
		ce(LOW);
	}
}

bool RF24::rxFifoFull()
{
	return read_register(FIFO_STATUS) & _BV(RX_FULL);
}

bool RF24::txStandBy()
{
#if defined(RF24_FAILURE_HANDLING)
	OneShotFastMs timer(TIMEOUT_MS);
#endif

	while(!(read_register(FIFO_STATUS) & _BV(TX_EMPTY))) {
		if(get_status() & _BV(MAX_RT)) {
			write_register(NRF_STATUS, _BV(MAX_RT));
			ce(LOW);
			flush_tx(); // Non blocking, flush the data
			return 0;
		}
#if defined(RF24_FAILURE_HANDLING)
		if(timer.expired()) {
			errNotify();
			return false;
		}
#endif
	}

	ce(LOW); // Set STANDBY-I mode
	return 1;
}

bool RF24::txStandBy(uint32_t timeout, bool startTx)
{
	if(startTx) {
		stopListening();
		ce(HIGH);
	}

	OneShotFastMs timer(timeout);

	while(!(read_register(FIFO_STATUS) & _BV(TX_EMPTY))) {
		if(get_status() & _BV(MAX_RT)) {
			write_register(NRF_STATUS, _BV(MAX_RT));
			ce(LOW); // Set re-transmit
			ce(HIGH);
			if(timer.expired()) {
				ce(LOW);
				flush_tx();
				return false;
			}
		}
#if defined(RF24_FAILURE_HANDLING)
		if(timer.elapsedTicks() > timer.checkTime<TIMEOUT_MS>()) {
			errNotify();
			return false;
		}
#endif
	}

	ce(LOW); // Set STANDBY-I mode
	return true;
}

void RF24::maskIRQ(bool tx, bool fail, bool rx)
{
	/* clear the interrupt flags */
	config_reg &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config_reg |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	write_register(NRF_CONFIG, config_reg);
}

uint8_t RF24::getDynamicPayloadSize()
{
	uint8_t result = spiTrans(R_RX_PL_WID);

	if(result > 32) {
		flush_rx();
		delay(2);
		result = 0;
	}

	return result;
}

bool RF24::available(uint8_t* pipe_num)
{
	if(read_register(FIFO_STATUS) & _BV(RX_EMPTY)) {
		return false;
	}

	// If the caller wants the pipe number, include that
	if(pipe_num != nullptr) {
		uint8_t status = get_status();
		*pipe_num = (status >> RX_P_NO) & 0x07;
	}

	return true;
}

void RF24::read(void* buf, uint8_t len)
{
	// Fetch the payload
	read_payload(buf, len);

	// Clear the two possible interrupt flags with one command
	write_register(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));
}

void RF24::whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready)
{
	// Read the status & reset the status in one easy call
	// Or is that such a good idea?
	uint8_t status = write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Report to the user what happened
	tx_ok = status & _BV(TX_DS);
	tx_fail = status & _BV(MAX_RT);
	rx_ready = status & _BV(RX_DR);
}

void RF24::openWritingPipe(uint64_t value)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

	write_register(RX_ADDR_P0, &value, addr_width);
	write_register(TX_ADDR, &value, addr_width);

	// const uint8_t max_payload_size = 32;
	// write_register(RX_PW_P0,min(payload_size,max_payload_size));
	write_register(RX_PW_P0, payload_size);
}

void RF24::openWritingPipe(const uint8_t* address)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.
	write_register(RX_ADDR_P0, address, addr_width);
	write_register(TX_ADDR, address, addr_width);

	// const uint8_t max_payload_size = 32;
	// write_register(RX_PW_P0,min(payload_size,max_payload_size));
	write_register(RX_PW_P0, payload_size);
}

static uint8_t child_pipe(uint8_t child)
{
	return RX_ADDR_P0 + child;
}

static uint8_t child_payload_size(uint8_t child)
{
	return RX_PW_P0 + child;
}

void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if(child == 0) {
		memcpy(pipe0_reading_address, &address, addr_width);
	}

	if(child <= 5) {
		// For pipes 2-5, only write the LSB
		if(child < 2) {
			write_register(child_pipe(child), &address, addr_width);
		} else {
			write_register(child_pipe(child), &address, 1);
		}

		write_register(child_payload_size(child), payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(child_pipe_enable(child)));
	}
}

void RF24::setAddressWidth(uint8_t a_width)
{
	if(a_width -= 2) {
		write_register(SETUP_AW, a_width % 4);
		addr_width = (a_width % 4) + 2;
	} else {
		write_register(SETUP_AW, 0);
		addr_width = 2;
	}
}

void RF24::openReadingPipe(uint8_t child, const uint8_t* address)
{
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if(child == 0) {
		memcpy(pipe0_reading_address, address, addr_width);
	}
	if(child <= 5) {
		// For pipes 2-5, only write the LSB
		if(child < 2) {
			write_register(child_pipe(child), address, addr_width);
		} else {
			write_register(child_pipe(child), address, 1);
		}
		write_register(child_payload_size(child), payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(child_pipe_enable(child)));
	}
}

void RF24::closeReadingPipe(uint8_t pipe)
{
	write_register(EN_RXADDR, read_register(EN_RXADDR) & ~_BV(child_pipe_enable(pipe)));
}

void RF24::toggle_features()
{
	packet.prepare();
	outbuf[0] = ACTIVATE;
	outbuf[1] = 0x73;
	packet.out.set(outbuf, 2);
	spidev.execute(packet);
}

void RF24::enableDynamicPayloads()
{
	// Enable dynamic payload throughout the system

	// toggle_features();
	write_register(FEATURE, read_register(FEATURE) | _BV(EN_DPL));

	debug_i("FEATURE=%i", read_register(FEATURE));

	// Enable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	write_register(DYNPD, read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) |
							  _BV(DPL_P0));

	dynamic_payloads_enabled = true;
}

void RF24::disableDynamicPayloads()
{
	// Disables dynamic payload throughout the system.  Also disables Ack Payloads

	// toggle_features();
	write_register(FEATURE, 0);

	debug_i("FEATURE=%u", read_register(FEATURE));

	// Disable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	write_register(DYNPD, 0);

	dynamic_payloads_enabled = false;
	ack_payloads_enabled = false;
}

void RF24::enableAckPayload()
{
	//
	// enable ack payload and dynamic payload features
	//

	// toggle_features();
	write_register(FEATURE, read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

	debug_i("FEATURE=%u", read_register(FEATURE));

	//
	// Enable dynamic payload on pipes 0 & 1
	//
	write_register(DYNPD, read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
	dynamic_payloads_enabled = true;
	ack_payloads_enabled = true;
}

void RF24::enableDynamicAck()
{
	//
	// enable dynamic ack features
	//
	// toggle_features();
	write_register(FEATURE, read_register(FEATURE) | _BV(EN_DYN_ACK));

	debug_i("FEATURE=%u", read_register(FEATURE));
}

void RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
	uint8_t data_len = std::min(len, uint8_t(32));

	packet.prepare();
	outbuf[0] = W_ACK_PAYLOAD | (pipe & 0x07);
	assert(len < sizeof(outbuf));
	memcpy(&outbuf[1], buf, len);
	packet.out.set(outbuf, 1 + len);
	packet.in.clear();
	spidev.execute(packet);
}

bool RF24::isPVariant()
{
	auto dR = getDataRate();
	bool result = setDataRate(RF24_250KBPS);
	setDataRate(dR);
	return result;
}

void RF24::setAutoAck(bool enable)
{
	write_register(EN_AA, enable ? 0x3F : 0x00);
}

void RF24::setAutoAck(uint8_t pipe, bool enable)
{
	if(pipe <= 6) {
		uint8_t en_aa = read_register(EN_AA);
		if(enable) {
			en_aa |= _BV(pipe);
		} else {
			en_aa &= ~_BV(pipe);
		}
		write_register(EN_AA, en_aa);
	}
}

bool RF24::testCarrier()
{
	return read_register(CD) & 0x01;
}

bool RF24::testRPD()
{
	return read_register(RPD) & 0x01;
}

void RF24::setPALevel(uint8_t level, bool lnaEnable)
{
	uint8_t setup = read_register(RF_SETUP) & 0xF8;

	if(level > 3) {
		// Invalid level, go to max PA
		level = (RF24_PA_MAX << 1) + lnaEnable; // +1 to support the SI24R1 chip extra bit
	} else {
		level = (level << 1) + lnaEnable;
	}

	write_register(RF_SETUP, setup |= level);
}

uint8_t RF24::getPALevel()
{
	return read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) >> 1;
}

uint8_t RF24::getARC()
{
	return read_register(OBSERVE_TX) & 0x0F;
}

bool RF24::setDataRate(rf24_datarate_e speed)
{
	uint8_t setup = read_register(RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

#if !defined(F_CPU) || F_CPU > 20000000
	txDelay = 250;
#else // 16Mhz Arduino
	txDelay = 85;
#endif
	if(speed == RF24_250KBPS) {
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= _BV(RF_DR_LOW);
#if !defined(F_CPU) || F_CPU > 20000000
		txDelay = 450;
#else // 16Mhz Arduino
		txDelay = 155;
#endif
	} else {
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if(speed == RF24_2MBPS) {
			setup |= _BV(RF_DR_HIGH);
#if !defined(F_CPU) || F_CPU > 20000000
			txDelay = 190;
#else // 16Mhz Arduino
			txDelay = 65;
#endif
		}
	}
	write_register(RF_SETUP, setup);

	// Read back register to confirm
	return read_register(RF_SETUP) == setup;
}

rf24_datarate_e RF24::getDataRate()
{
	uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	rf24_datarate_e result;
	if(dr == _BV(RF_DR_LOW)) {
		// '10' = 250KBPS
		result = RF24_250KBPS;
	} else if(dr == _BV(RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

void RF24::setCRCLength(rf24_crclength_e length)
{
	config_reg &= ~(_BV(CRCO) | _BV(EN_CRC));

	// switch uses RAM (evil!)
	if(length == RF24_CRC_DISABLED) {
		// Do nothing, we turned it off above.
	} else if(length == RF24_CRC_8) {
		config_reg |= _BV(EN_CRC);
	} else {
		config_reg |= _BV(EN_CRC);
		config_reg |= _BV(CRCO);
	}
	write_register(NRF_CONFIG, config_reg);
}

rf24_crclength_e RF24::getCRCLength()
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
	uint8_t AA = read_register(EN_AA);
	config_reg = read_register(NRF_CONFIG);

	if((config_reg & _BV(EN_CRC)) || (AA != 0)) {
		if(config_reg & _BV(CRCO)) {
			result = RF24_CRC_16;
		} else {
			result = RF24_CRC_8;
		}
	}

	return result;
}

void RF24::disableCRC()
{
	config_reg &= ~_BV(EN_CRC);
	write_register(NRF_CONFIG, config_reg);
}

void RF24::setRetries(uint8_t delay, uint8_t count)
{
	write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

void RF24::startConstCarrier(rf24_pa_dbm_e level, uint8_t channel)
{
	write_register(RF_SETUP, read_register(RF_SETUP) | _BV(CONT_WAVE));
	write_register(RF_SETUP, read_register(RF_SETUP) | _BV(PLL_LOCK));
	setPALevel(level);
	setChannel(channel);
	debug_i("RF_SETUP=%02x", read_register(RF_SETUP));
	ce(HIGH);
}

void RF24::stopConstCarrier()
{
	write_register(RF_SETUP, read_register(RF_SETUP) & ~_BV(CONT_WAVE));
	write_register(RF_SETUP, read_register(RF_SETUP) & ~_BV(PLL_LOCK));
	ce(LOW);
}
