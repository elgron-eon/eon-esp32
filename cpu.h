/*
 * cpu.h
 *
 * eon cpu
 * (c) JCGV, junio del 2022
 *
 */
// eon state
static struct {
    word32  r[16];
    word32  pc;
    word32  s[16];
    word32  irq_pending;
    byte    st;
    byte    rst;
    byte    trace;

    // fetch state
    word16  op;
    word32  imm;
} eon;

// special registers
enum {
    REG_MOD, REG_PAGETAB, REG_STATUS, REG_CYCLE,
    REG_IRQ_MASK, REG_IRQ_PC, REG_IRQ_SCRATCH, REG_IRQ_SAVE,
    REG_SYS_0,	  REG_SYS_PC, REG_SYS_SCRATCH, REG_SYS_SAVE,
    REG_EXC_CODE, REG_EXC_PC, REG_EXC_SCRATCH, REG_EXC_SAVE,
    NSPECIAL
};

// exceptions
enum {
    E_DECODE, E_ILLEGAL, E_AUTH, E_ALIGN, E_IRET, E_ZERODIV
};

// states
enum {
    S_RESET, S_RWAIT, S_HALT, S_DONE, S_EXC,
    S_WAIT, S_FETCH, S_IMM2, S_IMM4H, S_IMM4L,
    S_EXEC
};

// cpu mode bits
enum {MODE_SYS = 0, MODE_USER = 1, MODE_IRQ = 2};

// irq mask
enum {IRQ_RTC = 1, IRQ_TIMER = 2, IRQ_CON = 4, IRQ_DISK = 8};

static void eon_irq (unsigned i) {
    eon.irq_pending |= i;
}

// reset
static void eon_reset (void) {eon.st = S_RESET;}

// halt
static void eon_halt (void) {eon.st = S_HALT;}

// excpetions
static void eon_raise (unsigned code) {
    eon.st		= S_EXC;
    eon.s[REG_EXC_CODE] = code;
}

// cycle state machine
static void eon_go (unsigned ncycles) {
    // check reset
    if (eon.st > S_RWAIT) {
#if 1 //def BOARD_ESP8266
	int rc = HIGH;
#else
	int rc = digitalRead (zRESET);
#endif
	if (rc == LOW) {
	    eon.st = S_RESET;
	    //Serial.println ("RESET !!!");
	}
    }

    // step
    //sprintf (sbuf, "\t************** %u cycles", ncycles); Serial.println (sbuf);
    for (; ncycles > 0; ncycles--, eon.s[REG_CYCLE]++) {
    //sprintf (sbuf, "\t %04x ST%u PC%04lx", ncycles, eon.st, eon.pc); Serial.println (sbuf);
    switch (eon.st) {
	case S_RESET:
	    eon.rst   = RESETCOUNT - 1;
	    eon.pc    = 0;
	    eon.trace = 0;
	    eon.st    = S_RWAIT;
	    eon.s[REG_STATUS]	= MODE_SYS;
	    eon.s[REG_IRQ_MASK] = 0;
	    eon.s[REG_CYCLE]	= 0;
	    dcache_init  ();
	    digitalWrite (zRST, HIGH);
	    digitalWrite (zRUN, LOW);
	    digitalWrite (zBAD, LOW);
	    break;
	case S_RWAIT:
	    if (eon.rst == 0) {
		eon.st = S_FETCH;
		digitalWrite (zRST, LOW);
		digitalWrite (zRUN, HIGH);
	    } else {
		eon.rst--;
		delay (DELAYMS);
	    }
	    break;
	case S_HALT:
	    digitalWrite (zRUN, LOW);
	    eon.st = S_DONE;
	    sprintf (sbuf, "\n\t| emulation stop at %08lx: %lu cycles. R0=%08lx", eon.pc, eon.s[REG_CYCLE], eon.r[0]);
	    Serial.println (sbuf);
	    sprintf (sbuf, "\t| dache: %u*%u lines %lu requests %lu hits %lu loads %lu evicted %lu writeback",
		NLINES, WAYS, dcache_use, dcache_hit, dcache_load, dcache_evicted, dcache_wback
		);
	    Serial.println (sbuf);
	    break;
	case S_DONE:
	    break;
	case S_EXC:
	    digitalWrite (zBAD, HIGH);
	    if (0) {
		eon.st = S_HALT;
		sprintf (sbuf, "\t| exception %lx at %lx", eon.s[REG_EXC_CODE], eon.pc); Serial.println (sbuf);
	    } else {
		eon.s[REG_EXC_SAVE] = eon.pc | (eon.s[REG_STATUS] & 1); // USER/SYS mode bit
		eon.s[REG_STATUS]  &= ~1;   // SYSMODE
		eon.pc		    = eon.s[REG_EXC_PC];
		eon.st		    = S_FETCH;
	    }
	    break;
	case S_WAIT:
	    // wait for irq
	    if (eon.irq_pending)
		eon.st = S_FETCH;
	    else {
		idle ();
		return;
	    }
	    break;
	case S_FETCH:
	    if ((eon.s[REG_STATUS] & MODE_IRQ) == 0 && eon.irq_pending && (eon.s[REG_IRQ_MASK] & eon.irq_pending)) {
		eon.s[REG_IRQ_SAVE] = eon.pc | (eon.s[REG_STATUS] & 1); // USER/SYS mode bit
		eon.s[REG_STATUS]  |= MODE_IRQ;
		eon.pc		    = eon.s[REG_IRQ_PC];
	    }
	    eon.op  = dcache (eon.pc, false, 2, 0);
	    eon.imm = 0;
	    //sprintf (sbuf, "\t| FETCH at %lx = %x", eon.pc, eon.op); Serial.println (sbuf);
	    eon.pc += 2;

	    // decode opcode size
	    switch (eon.op >> 12) {
		case  0:
		    if (((eon.op >> 8) & 0x0f) == 0x0f) {
			unsigned op = eon.op & 0x0f;
			eon.st = op >= 0xc ? S_IMM4H : op >= 0x8 ? S_IMM2 : S_EXEC;
		    } else
			eon.st = S_EXEC;
		    break;
		case  1:
		case  2:
		case  3: eon.st = S_IMM2; break;
		default: eon.st = S_EXEC; break;
	    }
	    break;
	case S_IMM2:
	    eon.imm = dcache (eon.pc, false, 2, 0);
	    eon.imm = SEXT16 (eon.imm);
	    eon.pc += 2;
	    eon.st  = S_EXEC;
	    break;
	case S_IMM4H:
	    eon.imm = dcache (eon.pc, false, 2, 0);
	    eon.imm <<= 16;
	    eon.pc += 2;
	    eon.st  = S_IMM4L;
	    break;
	case S_IMM4L:
	    eon.imm   |= dcache (eon.pc, false, 2, 0) & 0x0fffful;
	    eon.pc    += 2;
	    eon.st     = S_EXEC;
	    break;
	case S_EXEC: {
		// debug
		//if (eon.pc == 0x026c) eon.trace = 1;
		//if (eon.pc == 0x02a6) eon.trace = 0;
		if (0) {
		    sprintf (sbuf, "\t| %04lX EXEC %04X %08lX", eon.pc, eon.op, eon.imm);
		    Serial.print (sbuf);
		    for (unsigned i = 0; i < 16; i++) {
			sprintf (sbuf, "%sR%02u=%08lx", i % 4 == 0 ? "\n\t| " : " ", i, eon.r[i]);
			Serial.print (sbuf);
		    }
		    Serial.print ("\n");
		}

		// continue by default
		eon.st = S_FETCH;

		// exec
		#define REGAT(pos)  (eon.op >> pos) & 0x0f
		#define SP	    15
		unsigned k = eon.op >> 12;
		switch (k) {
		    case  0: {
			    unsigned rd = REGAT (8);
			    unsigned rs = REGAT (4);
			    unsigned fn = REGAT (0);
			    if (rd == SP) {
				if (rs == SP)
				    switch (fn) {
					case 0x0:   // illegal
					    eon_raise (E_ILLEGAL);
					    break;
					case 0x3:   // wait
					    eon.st = S_WAIT;
					    break;
					case 0x4:   // iret
					    if (eon.s[REG_STATUS] & MODE_IRQ) {
						word32 s = eon.s[REG_IRQ_SAVE];
						eon.pc	 = s & ~1;
						eon.s[REG_STATUS] = s & 1;
					    } else
						eon_raise (E_IRET);
					    break;
					case 0x8:   // enter
					    eon.r[SP] = LIMIT32 (eon.r[SP] + eon.imm * 8);
					    break;
					case 0xc:   // jmp i32
					    eon.pc = LIMIT32 (eon.pc + (eon.imm * 2));
					    break;
					case 0xd:   // jal i32
					    eon.r[14] = eon.pc;
					    eon.pc    = LIMIT32 (eon.pc + (eon.imm * 2));
					    break;
					default:
					    eon_raise (E_DECODE);
					    break;
				    }
				else
				    switch (fn) {
					case 0x0:   // jmpr
					    eon.pc = eon.r[rs];
					    break;
					case 0x4:   // istat
					    eon.r[rs]	    = eon.irq_pending;
					    eon.irq_pending = 0;
					    break;
					case 0x8:   // get
					    eon.r[rs] = eon.s[eon.imm & 0x0f];
					    break;
					case 0x9:   // set
					    eon.s[eon.imm & 0x0f] = eon.r[rs];
					    break;
					case 0xb:   // inv
					    dcache_inv (eon.r[rs], eon.imm);
					    break;
					case 0xc:   // li
					    eon.r[rs] = LIMIT32 (eon.imm);
					    break;
					case 0xd:   // lea
					    eon.r[rs] = LIMIT32 (eon.pc + eon.imm);
					    break;
					default:
					    eon_raise (E_DECODE);
					    break;
				    }
			    } else {
				word32 sv = rs == SP ? 0 : eon.r[rs];
				switch (fn) {
				    case 0x8:	// csetz
					eon.r[rd] = sv == 0 ? 1 : 0;
					break;
				    case 0x9:	// csetnz
					eon.r[rd] = sv == 0 ? 0 : 1;
					break;
				    case 0xa:	// csetn
					eon.r[rd] = ((int32_t) sv) < 0 ? 1 : 0;
					break;
				    case 0xb:	// csetnn
					eon.r[rd] = ((int32_t) sv) < 0 ? 0 : 1;
					break;
				    case 0xc:	// csetp
					eon.r[rd] = ((int32_t) sv) > 0 ? 1 : 0;
					break;
				    case 0xd:	// csetnp
					eon.r[rd] = ((int32_t) sv) > 0 ? 0 : 1;
					break;
				    case 0xe:	// in
					eon.r[rd] = cpu_in (sv);
					break;
				    case 0xf:	// out
					cpu_out (sv, eon.r[rd]);
					break;
				    default: eon_raise (E_DECODE); break;
				}
			    }
			} break;
		    case  1: {
			    unsigned rd = REGAT (8);
			    unsigned rs = REGAT (4);
			    unsigned fn = REGAT (0);
			    if (fn < 8) {
				// load
				int   sext = fn & 1;
				word32 nb  = 1 << ((fn >> 1) & 3);
				word32 ptr = LIMIT32 (eon.r[rs] + eon.imm * nb);
				if ((ptr & ~(nb - 1)) != ptr) {
				    eon_raise (E_ALIGN);
				    break;
				}

				word32 v = dcache (ptr, false, nb, 0);
				switch (nb) {
				    case  1: v = sext ? SEXT8  (v) : v & 0x0ff; break;
				    case  2: v = sext ? SEXT16 (v) : v & 0x0ffff; break;
				    default: break;
				}
				eon.r[rd] = LIMIT32 (v);
			    } else if (fn < 12) {
				// store
				word32 nb  = 1 << (fn - 8);
				word32 ptr = LIMIT32 (eon.r[rs] + eon.imm * nb);
				if ((ptr & ~(nb - 1)) != ptr) {
				    eon_raise (E_ALIGN);
				    break;
				}
				dcache (ptr, true, nb, eon.r[rd]);
			    } else {
				eon_raise (E_DECODE);
			    }
			} break;
		    case  2: {
			    unsigned rl = REGAT (8);
			    unsigned rr = REGAT (4);
			    unsigned fn = REGAT (0);
			    word32   lv = rl == SP ? 0 : eon.r[rl];
			    word32   rv = rr == SP ? 0 : eon.r[rr];
			    int  branch = 0;
			    switch (fn) {
				case 0x0: branch = lv == rv; break;
				case 0x1: branch = lv != rv; break;
				case 0x2: branch = lv <  rv; break;
				case 0x3: branch = (long) (int32_t) lv < (long) (int32_t) rv; break;
				case 0x4: branch = lv <= rv; break;
				case 0x5: branch = (long) (int32_t) lv <= (long) (int32_t) rv; break;
				default : eon_raise (E_DECODE); break;
			    }
			    if (branch) eon.pc = LIMIT32 (eon.pc + eon.imm * 2);
			} break;
		    case  3: {
			    unsigned rd = REGAT (8);
			    unsigned rl = REGAT (4);
			    unsigned fn = REGAT (0);
			    word32 lv = rl == SP ? 0 : eon.r[rl];
			    word32 rv = eon.imm;
			    word32 dv = 0;
			    switch (fn) {
				case 0x4: dv = lv + rv; break;
				case 0x5: dv = lv - rv; break;
				case 0x6: dv = lv * rv; break;
				case 0x7: if (!rv)
					     eon_raise (E_ZERODIV);
					  else {
					    dv		   = lv / rv;
					    eon.s[REG_MOD] = lv % rv;
					  }
					  break;
				case 0x8: dv = lv & rv; break;
				case 0x9: dv = lv | rv; break;
				case 0xa: dv = lv ^ rv; break;
				case 0xb: dv = lv << rv; break;
				case 0xc: dv = lv >> rv; break;
				case 0xd: dv = LIMIT32 (((int32_t) lv) >> rv); break;
				default : eon_raise (E_DECODE); break;
			    }
			    eon.r[rd] = LIMIT32 (dv);
			} break;
		    default: {
			    unsigned rd = REGAT (8);
			    unsigned rl = REGAT (4);
			    unsigned rr = REGAT (0);
			    word32 lv = rl == SP ? 0 : eon.r[rl];
			    word32 rv = eon.r[rr];
			    word32 dv = 0;
			    switch (k) {
				case 0x4: dv = lv + rv; break;
				case 0x5: dv = lv - rv; break;
				case 0x6: dv = lv * rv; break;
				case 0x7: if (!rv)
					     eon_raise (E_ZERODIV);
					  else {
					    dv		   = lv / rv;
					    eon.s[REG_MOD] = lv % rv;
					  }
					  break;
				case 0x8: dv = lv & rv; break;
				case 0x9: dv = lv | rv; break;
				case 0xa: dv = lv ^ rv; break;
				case 0xb: dv = lv << rv; break;
				case 0xc: dv = lv >> rv; break;
				case 0xd: dv = LIMIT32 (((int32_t) lv) >> rv); break;
				default : eon_raise (E_DECODE); break;
			    }
			    eon.r[rd] = LIMIT32 (dv);
			} break;
		}
	    } break;
    }
    }
}
