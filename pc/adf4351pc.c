#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm/termbits.h>
#include <assert.h>

int serialport;
int adf_command(const uint32_t data) {
	ssize_t wr, txi;
	uint8_t tx[5];
	tx[0] = 'R';
	tx[1] = 0xFF & (data >> 24);
	tx[2] = 0xFF & (data >> 16);
	tx[3] = 0xFF & (data >> 8);
	tx[4] = 0xFF & (data);
	txi = 0;
	while(txi < 5) {
		wr = write(serialport, tx+txi, 5-txi);
		if(wr <= 0) {
			perror("write to serial port");
			return -1;
		} else {
			txi += wr;
		}
	}
	return 0;
}


void adf_write_registers(uint32_t *regs) {
	int i;
	// correct sequence to write registers
	for(i = 5; i >= 0; i--) {
		adf_command(regs[i] | i);
		usleep(100000);
	}
}

uint32_t r_int = 43, r_frac = 1255;

void adf_init() {
	/*
	Modulus must be below 4095.
	Using R counter of 3 on 10 MHz reference allows nearly
	1200 Hz step by using a modulus of 2778:
	10e6 / 3 / 2778 = 1200.336
	*/
	uint32_t regs[6];
	uint32_t
	/*r_int = 43,
	r_frac = 1255,*/
	r_phase_adjust = 0,
	r_prescaler = 0,
	r_phase = 0,
	r_modulus = 2778,
	
	r_lownoise = 0,
	r_muxout = 0,
	r_refdoubler = 0,
	r_rdiv2 = 0,
	r_rcount = 4,
	r_doublebuffer = 0,
	r_cp_current = 15, // 0 = minimum, 15 = maximum
	r_ldf = 0, // lock detect: 0 for frac-N, 1 for int-N
	r_ldp = 0,
	r_pd_polarity = 1,
	r_powerdown = 0,
	r_cp_3state = 0,
	r_counter_reset = 0,
	
	r_bandsel_clkmode = 0,
	r_abp = 0, // 0 for frac-N, 1 improves integer-N
	r_charge_cancel = 0, // 0 for frac-N, 1 for int-N
	r_csr = 0, // 1 improves lock time, must set minimum charge pump current
	r_clkdivmode = 1,
	r_clkdiv = 0,
	
	r_feedback_sel = 0, // 0 from divider, 1 from VCO
	r_rf_div = 5, // 2**x
	r_bandsel_clkdiv = 10, // ?
	r_vco_powerdown = 0,
	r_mtld = 0,
	r_auxout = 0,
	r_auxout_en = 0,
	r_auxout_power = 0,
	r_rfout_en = 1,
	r_outpower = 3,   // 3 = maximum
	r_ld_pin_mode = 1 // 1 = lock detect
	;
	assert(r_int < 1<<16);
	assert(r_frac < 1<<12);
	assert(r_modulus < 1<<12);
	assert(r_rcount < 1<<10);
	assert(r_cp_current < 1<<4);
	regs[0] = (r_int << 15) | (r_frac << 3);
	regs[1] = (r_phase_adjust << 28) | (r_prescaler << 27) |
	          (r_phase << 15) | (r_modulus << 3);
	regs[2] = (r_lownoise << 29) | (r_muxout << 26) |
	          (r_refdoubler << 25) | (r_rdiv2 << 24) |
	          (r_rcount << 14) | (r_doublebuffer << 13) |
	          (r_cp_current << 9) | (r_ldf << 8) | (r_ldp << 7) |
	          (r_pd_polarity << 6) | (r_powerdown << 5) |
	          (r_cp_3state << 4) | (r_counter_reset << 3);
	regs[3] = (r_bandsel_clkmode << 23) | (r_abp << 22) |
	          (r_charge_cancel << 21) | (r_csr << 18) |
	          (r_clkdivmode << 15) | (r_clkdiv << 3);
	regs[4] = (r_feedback_sel << 23) | (r_rf_div << 20) |
	          (r_bandsel_clkdiv << 12) |
	          (r_vco_powerdown << 11) | (r_mtld << 10) |
	          (r_auxout << 9) | (r_auxout_en << 8) |
	          (r_auxout_power << 6) | (r_rfout_en << 5) |
	          (r_outpower << 3);
	regs[5] = (r_ld_pin_mode << 22);
	adf_write_registers(regs);

	usleep(100000);
	// enable phase adjust to disable band selection
	r_phase_adjust = 1;
	regs[1] = (r_phase_adjust << 28) | (r_prescaler << 27) |
	          (r_phase << 15) | (r_modulus << 3);
	
	adf_write_registers(regs);
}


#define RXBUF 15
#define TXBUF 16
int main(int argc, char *argv[]) {
	ssize_t r, i;
	uint8_t rxbuf[RXBUF], txbuf[TXBUF];
	int serialspeed;
	int inpipe=0;
	struct termios2 term2;

	if(argc < 3) return 1;
	serialspeed = atoi(argv[2]);
	serialport = open(argv[1], O_RDWR);
	if(serialport < 0) return 2;

	ioctl(serialport, TCGETS2, &term2);
	term2.c_cflag &= ~CBAUD;
	term2.c_cflag |= BOTHER;
	term2.c_ispeed = serialspeed;
	term2.c_ospeed = serialspeed;
	r = ioctl(serialport, TCSETS2, &term2);
	if(r < 0) perror("TCSETS2");

	// make input pipe nonblocking
	/*flags = fcntl(inpipe, F_GETFL, 0);
	fcntl(inpipe, F_SETFL, flags | O_NONBLOCK);*/

	adf_init();

	for(;;) {
		int overflowed = 0;
		r = read(serialport, rxbuf, RXBUF);
		//fprintf(stderr, "%zd ", r);
		if(r <= 0) {
			perror("read from serial port");
			goto err;
		}
		for(i = 0; i < r; i++) {
			if(rxbuf[r] == 'O') overflowed = 1;
		}
		if(overflowed) {
			fprintf(stderr, "Overflow in Arduino buffer!\n");
		}
		if(rxbuf[r-1] == 17) { // xon received last? ready to transmit
			ssize_t txmax;
			txmax = r+1; // we should send a bit more than one command per one received byte
			if(txmax >= TXBUF) txmax = TXBUF;

			r = read(inpipe, txbuf, txmax);
			if(r <= 0 && errno != EWOULDBLOCK) {
				perror("read from tx pipe");
				goto err;
			}
			for(i = 0; i < r; i++) {
				uint32_t r_frac_new;
				// change fractional part in synthesizer
				r_frac_new = r_frac + (3 & txbuf[i]);
				adf_command((r_int << 15) | (r_frac_new << 3) | 0);
			}
		}
	}
	err:
	return 0;
}

