extern const __supervisor_stack_stop: u32;
extern const __undefined_stack_stop: u32;
extern const __fiq_stack_stop: u32;
extern const __irq_stack_stop: u32;
extern const __abort_stack_stop: u32;
extern const __system_stack_stop: u32;
extern const __secure_stack_stop: u32;
extern const __user_stack_stop: u32;

extern const __exception_vectors: u32;

export fn _start() callconv(.naked) noreturn {
    asm volatile ("cps #0b10001");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__fiq_stack_stop),
    );

    asm volatile ("cps #0b10010");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__irq_stack_stop),
    );

    asm volatile ("cps #0b10111");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__abort_stack_stop),
    );

    asm volatile ("cps #0b11011");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__undefined_stack_stop),
    );

    asm volatile ("cps #0b11111");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__system_stack_stop),
    );

    asm volatile ("cps #0b10110");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__secure_stack_stop),
    );

    asm volatile ("cps #0b10011");
    asm volatile (
        \\ mov sp, %[val]
        :
        : [val] "{r0}" (&__supervisor_stack_stop),
    );
    asm volatile ("bl main");
}

const UART = struct {
    const PL011_BASE: u32 = 0x101f1000;

    const DR = packed struct(u32) {
        const offset = 0;
        data: u8,
        _1: u24,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTCR = packed struct(u32) {
        const offset = 0x030;
        uarten: bool,
        _1: u7,
        txe: bool,
        rxe: bool,
        _2: u22,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTIMSC = packed struct(u32) {
        const offset = 0x038;
        _1: u4,
        rxim: bool,
        txim: bool,
        _2: u26,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTLCR_H = packed struct(u32) {
        const offset = 0x02C;
        _1: u4,
        fen: bool,
        _2: u27,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTIFLS = packed struct(u32) {
        const offset = 0x034;
        tx: u3,
        rx: u3,
        _1: u26,

        fn getRxTriggerPoint(self: UARTIFLS, len: usize) usize {
            return switch (self.rx) {
                0 => @divTrunc(len, 8),
                1 => @divTrunc(len, 4),
                2 => @divTrunc(len, 2),
                3 => @divTrunc(len * 3, 4),
                4 => @divTrunc(len * 7, 8),
                else => 0,
            };
        }

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTIBRD = packed struct(u32) {
        const offset = 0x024;
        divint: u16,
        _1: u16,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTFBRD = packed struct(u32) {
        const offset = 0x028;
        divfrac: u6,
        _1: u26,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    const UARTFR = packed struct(u32) {
        const offset = 0x018;
        _1: u3,
        busy: bool,
        rxfe: bool,
        txff: bool,
        rxff: bool,
        txfe: bool,
        _2: u24,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + PL011_BASE);
        }
    };

    fn enableTransimit() void {
        UARTFBRD.getPtr().divfrac = 3;
        UARTIBRD.getPtr().divint = 26;

        UARTCR.getPtr().txe = true;
        UARTCR.getPtr().uarten = true;
    }

    fn enableReceive() void {
        UARTFBRD.getPtr().divfrac = 3;
        UARTIBRD.getPtr().divint = 26;

        UARTCR.getPtr().rxe = true;

        UARTIMSC.getPtr().rxim = true;

        UARTCR.getPtr().uarten = true;
    }

    fn busy() bool {
        return UARTFR.getPtr().busy;
    }

    fn receiveFIFOFull() bool {
        return UARTFR.getPtr().rxff;
    }

    fn receiveFIFOEmpty() bool {
        return UARTFR.getPtr().rxfe;
    }

    fn transmitFIFOFull() bool {
        return UARTFR.getPtr().txff;
    }

    fn transmitFIFOEmpty() bool {
        return UARTFR.getPtr().txfe;
    }

    fn disable() void {
        while (busy()) {}

        UARTCR.getPtr().txe = false;
        UARTCR.getPtr().rxe = false;

        UARTLCR_H.getPtr().fen = false;
        UARTIMSC.getPtr().rxim = false;
        UARTIMSC.getPtr().txim = false;

        UARTCR.getPtr().uarten = false;
    }

    fn writeChar(char: u8) void {
        DR.getPtr().data = char;
    }

    fn write(_: void, buf: []const u8) !usize {
        for (buf) |by| writeChar(by);
        return buf.len;
    }

    const Writer = std.io.GenericWriter(void, error{}, write){ .context = {} };

    fn print(comptime fmt: []const u8, args: anytype) void {
        std.fmt.format(Writer, fmt, args) catch unreachable;
    }
};

const VIC = struct {
    const base = 0x10140000;

    const VICIRQSTATUS = packed struct(u32) {
        const offset = 0;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const VICFIQSTATUS = packed struct(u32) {
        const offset = 4;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const VICRAWINTR = packed struct(u32) {
        const offset = 8;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const VICINTSELECT = packed struct(u32) {
        const offset = 12;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn selectFIQ(self: *@This(), n: u5) void {
            self.back |= (std.math.shl(u32, 1, n));
        }

        fn selectIRQ(self: *@This(), n: u5) void {
            self.back &= ~(std.math.shl(u32, 1, n));
        }
    };
    const VICINTENABLE = packed struct(u32) {
        const offset = 16;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn enable(self: *@This(), n: u5) void {
            self.back |= (std.math.shl(u32, 1, n));
        }

        fn disable(self: *@This(), n: u5) void {
            self.back &= ~(std.math.shl(u32, 1, n));
        }
    };
    const VICINTENCLEAR = packed struct(u32) {
        const offset = 20;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn disable(self: *@This(), n: u5) void {
            self.back |= (std.math.shl(u32, 1, n));
        }
    };
    const VICSOFTINT = packed struct(u32) {
        const offset = 24;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn trigger(self: *@This(), n: u5) void {
            self.back |= (std.math.shl(u32, 1, n));
        }
    };
    const VICSOFTINTCLEAR = packed struct(u32) {
        const offset = 28;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn unTrigger(self: *@This(), n: u5) void {
            self.back |= (std.math.shl(u32, 1, n));
        }
    };
    const VICPROTECTION = packed struct(u32) {
        const offset = 32;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn enable(self: *@This()) void {
            self.back = 1;
        }
    };
    const VICVECTADDR = packed struct(u32) {
        const offset = 0x30;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }

        fn read(self: *@This()) u32 {
            return self.back;
        }

        fn write(self: *@This(), addr: u32) void {
            self.back = addr;
        }
    };
    const VICDEFVECTADDR = packed struct(u32) {
        const offset = 0x34;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };

    const VICVECTADDR_ARRY = extern struct {
        const offset = 0x100;
        addrs: [16]u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };

    const VICVECTCNTL_ARRY = extern struct {
        const offset = 0x100;
        const VICVECTCNTL = packed struct(u32) { //
            intsource: u5,
            e: bool,
            _1: u26,

            fn enable(self: *@This()) void {
                self.e = true;
            }

            fn setIntSource(self: *@This(), src: u5) void {
                self.intsource = src;
            }
        };
        cntls: [16]VICVECTCNTL,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };

    inline fn enableInterrupts() void {
        asm volatile ("cpsie aif");
    }

    inline fn disableInterrupts() void {
        asm volatile ("cpsid aif");
    }
};

const RTC = struct {
    const base = 0x101e8000;

    const RTCDR = packed struct(u32) {
        const offset = 0;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCMR = packed struct(u32) {
        const offset = 4;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCLR = packed struct(u32) {
        const offset = 8;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCCR = packed struct(u32) {
        const offset = 12;

        rtc_start: bool,
        _1: u31,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCIMSC = packed struct(u32) {
        const offset = 16;

        enable_intr: bool,
        _1: u31,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCRIS = packed struct(u32) {
        const offset = 20;

        status: bool,
        _1: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCMIS = packed struct(u32) {
        const offset = 24;

        status: bool,
        _1: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const RTCICR = packed struct(u32) {
        const offset = 28;

        disable_intr: bool,
        _1: u31,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
};

const DualCounter = struct {
    const base = 0x101e2000;

    const Timer1Load = packed struct(u32) {
        const offset = 0;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };

    const Timer1Value = packed struct(u32) {
        const offset = 4;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const Timer1Control = packed struct(u32) {
        const offset = 8;

        const Mode1 = enum(u1) {
            wrapping = 0,
            oneshot,
        };

        const Mode2 = enum(u1) {
            free_running = 0,
            periodic,
        };

        const Size = enum(u1) {
            @"16",
            @"32",
        };

        mode1: Mode1,
        size: Size,
        _3: u3,
        int_enable: bool,
        mode2: Mode2,
        enable: bool,
        _1: u24,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const Timer1IntClr = packed struct(u32) {
        const offset = 12;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const Timer1RIS = packed struct(u32) {
        const offset = 16;

        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const Timer1MIS = packed struct(u32) {
        const offset = 20;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const TTimer1BGLoad = packed struct(u32) {
        const offset = 24;
        back: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
};

const MMCI = struct {
    const base = 0x10005000;

    const MCIPower = packed struct(u32) {
        const offset = 0;
        const Ctrl = enum(u2) {
            power_off,
            reserved,
            power_up,
            power_on,
        };
        ctrl: Ctrl,
        voltage: u4,
        open_drain: bool,
        rod: bool,
        _1: u24,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIClock = packed struct(u32) {
        const offset = 4;

        clkdiv: u8,
        enable: bool,
        power_save: bool,
        bypass: bool,
        _1: u21,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIArgument = packed struct(u32) {
        const offset = 8;

        cmd_arg: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MMCCommand = packed struct(u32) {
        const offset = 12;

        cmd_index: u6,
        response: bool,
        long_rsp: bool,
        interrupt: bool,
        pending: bool,
        enable: bool,
        _1: u21,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIRespCmd = packed struct(u32) {
        const offset = 16;

        resp_cmd: u6,
        _1: u26,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIResponse0 = extern struct {
        const offset = 20;
        resp: [4]u32,

        fn getShort() u32 {
            return @as(*@This(), @ptrFromInt(base + offset)).resp[0];
        }

        fn getLong() [16]u8 {
            const self = @as(*@This(), @ptrFromInt(base + offset));
            var cp: [16]u8 = @as(*[16]u8, @ptrCast(self)).*;
            std.mem.reverse(u8, cp[0..4]);
            std.mem.reverse(u8, cp[4..8]);
            std.mem.reverse(u8, cp[8..12]);
            std.mem.reverse(u8, cp[12..]);
            return cp;
            //return @as(*u128, @alignCast(@ptrCast(self))).*;
            //return @as(u128, self.resp[0]) << 96 |
            //    @as(u128, self.resp[1]) << 64 |
            //    @as(u128, self.resp[2]) << 32 | 
            //    @as(u128, self.resp[3] & ~@as(u32, 1));
        }

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    //const MCIResponse1 = packed struct(u32) {
    //    const offset = 24;
    //};
    //const MCIResponse2 = packed struct(u32) {
    //    const offset = 28;
    //};
    //const MCIResponse3 = packed struct(u32) {
    //    const offset = 32;
    //};
    const MCIDataTimer = packed struct(u32) {
        const offset = 36;
        data_time: u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIDataLength = packed struct(u32) {
        const offset = 40;

        data_length: u16,
        _1: u16,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIDataCtrl = packed struct(u32) {
        const offset = 44;
        const Direction = enum(u1) {
            to_card,
            from_card,
        };
        const TansferMode = enum(u1) {
            block,
            stream,
        };
        enable: bool,
        direction: Direction,
        mode: TansferMode,
        dma_enable: bool,
        block_size: u4,
        _1: u24,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIDataCnt = packed struct(u32) {
        const offset = 48;

        data_count: u16,
        _1: u16,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIStatus = packed struct(u32) {
        const offset = 52;

        cmd_crc_fail: bool,
        data_crc_fail: bool,
        cmd_time_out: bool,
        data_time_out: bool,
        tx_underrun: bool,
        rx_overrun: bool,
        cmd_resp_end: bool,
        cmd_sent: bool,
        data_end: bool,
        _1: bool,
        datablock_end: bool,
        //===10
        cmd_active: bool,
        tx_active: bool,
        rx_active: bool,
        tx_fifo_halfempty: bool,
        rx_fifo_halffull: bool,
        tx_fifo_full: bool,
        rx_fifo_full: bool,
        tx_fifo_empty: bool,
        rx_fifo_empty: bool,
        tx_data_avlbl: bool,
        rx_data_avlbl: bool,
        _2: u10,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIClear = packed struct(u32) {
        const offset = 56;

        cmd_crc_failclr: bool,
        data_crc_failclr: bool,
        cmd_time_outclr: bool,
        data_time_outclr: bool,
        tx_underrunclr: bool,
        rx_overrunclr: bool,
        cmd_resp_endclr: bool,
        cmd_sentclr: bool,
        data_endclr: bool,
        _1: bool,
        datablock_endclr: bool,

        _2: u21,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIMask0 = packed struct(u32) {
        const offset = 60;

        cmd_crc_fail: bool,
        data_crc_fail: bool,
        cmd_time_out: bool,
        data_time_out: bool,
        tx_underrun: bool,
        rx_overrun: bool,
        cmd_resp_end: bool,
        cmd_sent: bool,
        data_end: bool,
        _1: bool,
        datablock_end: bool,
        //===10
        cmd_active: bool,
        tx_active: bool,
        rx_active: bool,
        tx_fifo_halfempty: bool,
        rx_fifo_halffull: bool,
        tx_fifo_full: bool,
        rx_fifo_full: bool,
        tx_fifo_empty: bool,
        rx_fifo_empty: bool,
        tx_data_avlbl: bool,
        rx_data_avlbl: bool,
        _2: u10,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIMask1 = packed struct(u32) {
        const offset = 64;

        cmd_crc_fail: bool,
        data_crc_fail: bool,
        cmd_time_out: bool,
        data_time_out: bool,
        tx_underrun: bool,
        rx_overrun: bool,
        cmd_resp_end: bool,
        cmd_sent: bool,
        data_end: bool,
        _1: bool,
        datablock_end: bool,
        //===10
        cmd_active: bool,
        tx_active: bool,
        rx_active: bool,
        tx_fifo_halfempty: bool,
        rx_fifo_halffull: bool,
        tx_fifo_full: bool,
        rx_fifo_full: bool,
        tx_fifo_empty: bool,
        rx_fifo_empty: bool,
        tx_data_avlbl: bool,
        rx_data_avlbl: bool,
        _2: u10,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    const MCIFifoCnt = packed struct(u32) {
        const offset = 72;

        data_count: u15,
        _1: u17,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };
    /// The receive and transmit FIFOs can be read or written as 32-bit wide registers. The
    /// FIFOs contain 16 entries on 16 sequential addresses. This allows the microprocessor to
    /// use its load and store multiple operands to read/write to the FIFO. Table 3-20 shows the
    /// bit assignment of the MCIFIFO register.
    const MCIFIFO = extern struct {
        const offset = 0x80;

        data: [16]u32,

        inline fn getPtr() *@This() {
            return @ptrFromInt(offset + base);
        }
    };

    const CMD8 = struct {
        const ARG = packed struct(u32) { check_pattern: u8 = 0, vhs: u4, _1: u20 = 0 ,};
    };

    const ACMD41 = struct {
        const ARG = packed struct(u32){
            voltage_window: u24,
            s18r: bool = false,
            _1: u3 = 0,
            xpc: bool = true,
            _2: u1 = 0,
            hcs: bool = true,
            _3: u1 = 0
        };

        const RESP = packed struct (u32){
            
        };
    };

    const CardStatus = packed struct(u32) {
        const State = enum(u4) { idle, ready, ident, stdby, tran, data, rcv, prg, dis, _, };

        _1: u3,
        AKE_SEQ_ERROR: bool,
        _2: u1,
        APP_CMD: bool,
        FX_EVENT: bool,
        SWITCH_ERROR: bool,//===7
        READY_FOR_DATA: bool,//==8
        CURRENT_STATE: State,//==9
        ERASE_RESET: bool,//==12
        CARD_ECC_DISABLED: bool,
        WP_ERASE_SKIP: bool,
        CSD_OVERWRITE: bool,
        _3: u1,
        DEFERRED_RESPONSE: bool,
        ERROR: bool,
        CC_ERROR: bool,
        CARD_ECC_FAILED: bool,
        ILLEGAL_COMMAND: bool,
        COM_CRC_ERROR: bool,
        LOCK_UNLOCK_FAILED: bool,
        CARD_IS_LOCKED: bool,
        WP_VIOLATION: bool,
        ERASE_PARAM: bool,
        ERASE_SEQ_ERROR: bool,
        BLOCK_LEN_ERROR: bool,
        ADDRESS_ERROR: bool,
        OUT_OF_RANGE: bool,
    };

    const CARDREGS = struct {
        const OCR = packed struct(u32) {
            const Busy = enum(u1){initializing, done};
            const CCS = enum(u1){sdsc, sdhc_xc};


            _1: u24,
            s18a: bool,
            _2: u4,
            uhs2: bool,
            ccs: CCS,
            busy: Busy
        };

        const CID = packed struct(u128){
            _1: bool,
            crc: u7,
            mdrt: u12,
            _2: u4,
            psn: u32,
            prv: u8,
            pnm: u40,
            oid: u16,
            mid: u8,
        };
    };
};

/// baud rate divisor = 48_000_000
///                    ______________
///
///                     16 * 115200
///
///
/// i = 26
/// f = 3
///
///
export fn main() noreturn {
    UART.enableTransimit();
    //UART.enableReceive();

    //VIC.disableInterrupts();

    //VIC.enableInterrupts();
    //VIC.VICSOFTINT.getPtr().trigger(12);

    //asm volatile(".global breakpt");
    //asm volatile("breakpt:");
    //UART.enableReceive();

    //UART.disable();
    //UART.print("hello world!!!\n", .{});
    //asm volatile ("bkpt");
    MMCI.MCIPower.getPtr().ctrl = .power_on;

    MMCI.MCIArgument.getPtr().cmd_arg = 0;

    //MMCI.MMCCommand.getPtr().* = .{
    //    .enable = true,
    //    .interrupt = false,
    //    .pending = false,
    //    .long_rsp = false,
    //    .response = false,
    //    .cmd_index = 0,
    //    ._1 = 0
    //};

    MMCI.MCIDataTimer.getPtr().data_time = 0xffff_ffff;

    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = false,
        .cmd_index = 0,
        ._1 = 3,
    };

    MMCI.MCIArgument.getPtr().cmd_arg = @bitCast(MMCI.CMD8.ARG{ .vhs = 1 });

    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 8,
        ._1 = 0,
    };



    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    UART.print("hello world!!!: {}\n", .{@as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()))});


    MMCI.MCIArgument.getPtr().cmd_arg = 0;


    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 55,
        ._1 = 0,
    };

    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    UART.print("hello world!!!: {}\n", .{@as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()))});


    MMCI.MCIArgument.getPtr().cmd_arg = @bitCast(MMCI.ACMD41.ARG{
        .voltage_window = 0,
    });


    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 41,
        ._1 = 0,
    };

    MMCI.MCIClear.getPtr().* = @bitCast(@as(u32,0xffff_ffff));
    MMCI.MCIArgument.getPtr().cmd_arg = 0;


    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 55,
        ._1 = 0,
    };



    UART.print("---> hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    UART.print("---> hello world!!!: {}\n", .{@as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()))});


    MMCI.MCIClear.getPtr().* = @bitCast(@as(u32,0xffff_ffff));
    UART.print("====> zeroed: !!!: {}\n", .{MMCI.MCIStatus.getPtr().*});

    MMCI.MCIArgument.getPtr().cmd_arg = @bitCast(MMCI.ACMD41.ARG{
        .voltage_window = 0xff_ffff,
    });


    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 41,
        ._1 = 0,
    };

    //_= regg;
    const regg = @as(MMCI.CARDREGS.OCR, @bitCast(MMCI.MCIResponse0.getShort()));
    UART.print("hello world!!!: {}\n", .{regg.busy});

    UART.print("==========================================\n", .{});

    MMCI.MCIClear.getPtr().* = @bitCast(@as(u32,0xffff_ffff));
    MMCI.MCIArgument.getPtr().cmd_arg = 0;


    //MMCI.MMCCommand.getPtr().* = .{
    //    .enable = true,
    //    .interrupt = false,
    //    .pending = false,
    //    .long_rsp = false,
    //    .response = true,
    //    .cmd_index = 55,
    //    ._1 = 0,
    //};


    //UART.print("===---> hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    //UART.print("===---> hello world!!!: {b}\n", .{(MMCI.MCIResponse0.getShort()&0b1111000000000)});


    MMCI.MCIDataTimer.getPtr().data_time = 0xffff_ffff;


    MMCI.MCIArgument.getPtr().cmd_arg = 0;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = true,
        .response = true,
        .cmd_index = 2,
        ._1 = 0,
    };
//'X','Y', 'Q','E', 'M','U','!',
    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    
    //const cid = MMCI.MCIResponse0.getLong();

    //UART.print("cid: {x}\n", .{cid.prv});
    //UART.print("cid: {any}\n", .{cid});

    MMCI.MCIArgument.getPtr().cmd_arg = 0;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 3,
        ._1 = 0,
    };


    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});

    const rca = MMCI.MCIResponse0.getShort()&0xffff0000;

    UART.print("RCA: {}\n", .{
        rca>>16
    });


    MMCI.MCIArgument.getPtr().cmd_arg = MMCI.MCIResponse0.getShort();
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 7,
        ._1 = 0,
    };

    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});



    MMCI.MCIArgument.getPtr().cmd_arg =rca;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 13,
        ._1 = 0,
    };

    UART.print("=================> hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});

    var status = @as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()));
    UART.print("status: {}\n", .{status});

    MMCI.MCIClear.getPtr().* = @bitCast(@as(u32,0xffff_ffff));

//===========================write
    MMCI.MCIDataLength.getPtr().data_length = 512;
    MMCI.MCIDataCtrl.getPtr().* = .{
        ._1 = 0,
        .direction = .to_card,
        .enable = true,
        .dma_enable = false,
        .block_size = 9,
        .mode = .block
    };

    MMCI.MCIArgument.getPtr().cmd_arg = 0;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 24,
        ._1 = 0,
    };


    var written:usize = 0;

    while (written<512/4) {
        if(!MMCI.MCIStatus.getPtr().tx_fifo_full){
            MMCI.MCIFIFO.getPtr().data[0] = written*2;
            written += 1;
        }
    }

    UART.print("=======================writtten stuff============\n", .{});

//============================read
    MMCI.MCIDataLength.getPtr().data_length = 512;
    MMCI.MCIDataCtrl.getPtr().* = .{
        ._1 = 0,
        .direction = .from_card,
        .enable = true,
        .dma_enable = false,
        .block_size = 9,
        .mode = .block
    };


    MMCI.MCIArgument.getPtr().cmd_arg = 0;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 17,
        ._1 = 0,
    };

    status = @as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()));
    UART.print("status: {}\n", .{status});

    UART.print("hello world!!!: {}\n", .{MMCI.MCIStatus.getPtr().*});
    

    MMCI.MCIArgument.getPtr().cmd_arg =rca;
    MMCI.MMCCommand.getPtr().* = .{
        .enable = true,
        .interrupt = false,
        .pending = false,
        .long_rsp = false,
        .response = true,
        .cmd_index = 13,
        ._1 = 0,
    };


    status = @as(MMCI.CardStatus, @bitCast(MMCI.MCIResponse0.getShort()));
    UART.print("status: {}\n", .{status});


    var read: usize = 0;

    var acc: u32 = 0;

    while (read<512/4) {
        if(MMCI.MCIStatus.getPtr().rx_data_avlbl) {
            read += 1;
            acc = MMCI.MCIFIFO.getPtr().data[0];
            UART.print("===: {}\n", .{acc});
        }
    }

    UART.print("read finish...\n", .{});

    while (true) {
        //UART.print("=====================: {}\n", .{
        //    RTC.RTCDR.getPtr().back
        //});
    }
}
//0b1_0000_0000_0000
//4096
//0x600001d3;
//0x010

pub fn panic(message: []const u8, stack_trace: ?*std.builtin.StackTrace, n: ?usize) noreturn {
    _ = stack_trace;
    _ = n;
    UART.print("panic: {s}\n", .{message});
    while (true) {}
}

fn debugPanic(comptime fmt: []const u8, args: anytype) noreturn {
    UART.print(fmt, args);
    while (true) {}
}

export fn irqHandler(exception: Exception) void {
    //var arr: [1]u8 = undefined;

    const vector = VIC.VICVECTADDR.getPtr().read();

    //_ = exception;
    //while (true) {
    UART.print("Exception happened........ {}\n", .{ //
        exception,
    });

    outer: while (true) {
        var status = VIC.VICIRQSTATUS.getPtr().back;

        if (status == 0) break;

        for (0..32) |i| {
            if (status & 1 == 1) {
                if (i == 12) {
                    uartRXHandler();
                    VIC.VICSOFTINTCLEAR.getPtr().unTrigger(@truncate(i));
                } else if (i == 10) {
                    RTC.RTCICR.getPtr().disable_intr = true;
                    UART.print("clock: {}\n", .{RTC.RTCDR.getPtr().back});

                    RTC.RTCMR.getPtr().back += 5;
                } else if (i == 4) {
                    UART.print("timer1....\n", .{});
                    DualCounter.Timer1IntClr.getPtr().back = 0;
                } else {
                    debugPanic("unimplemented IRQ: {}\n", .{i});
                }
            }
            status >>= 1;

            if (status == 0) {
                break :outer;
            }
        }
    }

    VIC.VICVECTADDR.getPtr().back = vector;
}

fn uartRXHandler() void {
    while (!UART.UARTFR.getPtr().rxfe) {
        const data = UART.DR.getPtr().data;
        UART.print("uart rx data: {c}\n", .{data});
    }
}

export fn exceptionHandler(exception: Exception) void {
    //_ = exception;
    UART.print("{}\n", .{exception});

    VIC.VICVECTADDR.getPtr().back = 1;
    DualCounter.Timer1IntClr.getPtr().back = 0;

    while (true) {}
}

const Mode = enum(u5) { //
    user = 0b10000,
    fiq = 0b10001,
    irq = 0b10010,
    supervisor = 0b10011,
    abort = 0b10111,
    undefined = 0b11011,
    system = 0b11111,
    secure_monitor = 0b10110,
};

const Exception = enum(u8) { //
    svc,
    smc,
    undef,
    pabt,
    fiq,
    irq,
    dabt,
    bkpt,
};

export fn undefinedStub() callconv(.naked) void {
    asm volatile (std.fmt.comptimePrint("srsdb sp!, #{}", .{@as(u5, @intFromEnum(Mode.undefined))}));
    asm volatile ("push {r0-r12}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.undef))}));
    asm volatile ("blx exceptionHandler");
    asm volatile ("pop {r0-r12}");
    asm volatile ("rfeia sp!");
}

export fn svcStub() callconv(.naked) void {
    asm volatile (std.fmt.comptimePrint(
            \\  push {{r12}}
            \\  mrs r12, CPSR
            \\  and r12, r12, #0b11111
            \\  teq r12, #{}
            \\  pop {{r12}}
            \\  bne .skip
            \\  srsdb sp!, #{}
            \\  push {{r0-r12}}
            \\  mov r0, #{}
            \\  blx exceptionHandler
            \\  pop {{r0-r12}}
            \\  rfeia sp!

            // unreachable

            \\  .skip:
            \\  srsdb sp!, #{}
            \\  push {{r0-r12}}
            \\  mov r0, #{}
            \\  blx exceptionHandler
            \\  pop {{r0-r12}}
            \\  rfeia sp!
        , .{ //
            @as(u5, @intFromEnum(Mode.supervisor)),
            @as(u5, @intFromEnum(Mode.supervisor)),
            @as(u8, @intFromEnum(Exception.svc)),
            @as(u5, @intFromEnum(Mode.secure_monitor)),
            @as(u8, @intFromEnum(Exception.smc)),
        }));
}

export fn pabtStub() callconv(.naked) void {
    asm volatile ("sub r14, r14, #4");
    asm volatile (std.fmt.comptimePrint("srsdb sp!, #{}", .{@as(u5, @intFromEnum(Mode.abort))}));
    asm volatile ("push {r0-r12}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.pabt))}));
    asm volatile ("blx exceptionHandler");
    asm volatile ("pop {r0-r12}");
    asm volatile ("rfeia sp!");
}

export fn fiqStub() callconv(.naked) void {
    asm volatile ("sub r14, r14, #4");
    asm volatile (std.fmt.comptimePrint("srsdb sp!, #{}", .{@as(u5, @intFromEnum(Mode.fiq))}));
    asm volatile ("push {r0-r8}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.fiq))}));
    asm volatile ("blx exceptionHandler");
    asm volatile ("pop {r0-r8}");
    asm volatile ("rfeia sp!");
}

export fn irqStub() callconv(.naked) void {
    asm volatile ("sub r14, r14, #4");
    asm volatile (std.fmt.comptimePrint("srsfd sp!, #{}", .{@as(u5, @intFromEnum(Mode.irq))}));
    asm volatile ("push {r0-r12}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.irq))}));
    asm volatile ("blx irqHandler");
    asm volatile ("pop {r0-r12}");
    asm volatile ("rfefd sp!");
}

export fn dabtStub() callconv(.naked) void {
    asm volatile ("sub r14, r14, #8");
    asm volatile (std.fmt.comptimePrint("srsdb sp!, #{}", .{@as(u5, @intFromEnum(Mode.abort))}));
    asm volatile ("push {r0-r12}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.dabt))}));
    asm volatile ("blx exceptionHandler");
    asm volatile ("pop {r0-r12}");
    asm volatile ("rfeia sp!");
}

export fn bkptStub() callconv(.naked) void {
    asm volatile ("sub r14, r14, #4");
    asm volatile (std.fmt.comptimePrint("srsdb sp!, #{}", .{@as(u5, @intFromEnum(Mode.abort))}));
    asm volatile ("push {r0-r12}");
    asm volatile (std.fmt.comptimePrint("mov r0, #{}", .{@as(u8, @intFromEnum(Exception.bkpt))}));
    asm volatile ("blx exceptionHandler");
    asm volatile ("pop {r0-r12}");
    asm volatile ("rfeia sp!");
}

//export fn fiqHandler()

const std = @import("std");
