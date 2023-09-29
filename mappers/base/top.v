
`include "defs.v"

module top(
	inout [7:0]cpu_dat,
	input [14:0]cpu_addr,
	input cpu_ce, cpu_rw, m2,
	output cpu_irq,
	output cpu_dir, cpu_ex,
	
	inout [7:0]ppu_dat,
	input [13:0]ppu_addr,
	input ppu_oe, ppu_we, ppu_a13n,
	output ppu_ciram_ce, ppu_ciram_a10,
	output ppu_dir, ppu_ex,
	
	inout [7:0]prg_dat,
	output [22:0]prg_addr,
	output prg_ce, prg_oe, prg_we, prg_ub, prg_lb,
	
	inout [7:0]chr_dat,
	output [22:0]chr_addr,
	output chr_ce, chr_oe, chr_we, chr_ub, chr_lb,

	output srm_ce, srm_oe, srm_we,
	
	output spi_miso,
	input spi_mosi, spi_clk, spi_ss,
	
	input clk, fds_sw, mcu_busy,
	output led, pwm, fifo_rxf, boot_on,
	inout [3:0]gpio,
	inout [9:0]exp,
	output [2:0]xio,
	input rx,
	output tx
);

	
	//add: prg_dat, chr_dat, prg_addr[21], chr_addr[21], fifo_rxf, mcu_busy, gpio[3], exp
	//rem: map_on, gpio_3, core_io
	

	
	assign exp[0] = 1'bz;
	assign exp[2] = 1'bz;
	assign exp[5] = 1'bz;
	assign exp[6] = 1'bz;
	assign exp[7] = 1'bz;
	assign exp[9] = 1'bz;
	assign xio[2:0] = 3'bzzz;
	assign boot_on = 0;

	
	//----------------------------------------------------
	// EPSM Addressing
	//----------------------------------------------------  
    assign exp[1] = !(!cpu_rw & prg_ce & cpu_addr[14] & cpu_addr[13:2]==3'b00000000111);                // $401c-$401f
    assign exp[3] = 0; 
    assign exp[4] = prg_addr[1];
    assign exp[7] = prg_addr[0];
    assign exp[8] = m2;
	
	`include "sys_cfg_in.v"
	
	assign led = map_led | (sys_rst & map_rst & map_idx != 255);//ss_act
//**************************************************************************************** bus ctrl		
	wire eep_on, bus_conflicts, map_led, mem_dma, mir_4sc, map_ppu_oe, map_cpu_oe, prg_mem_oe, int_ciram_ce, int_ciram_a10;
	wire [7:0]map_cpu_dout, map_ppu_dout, eep_ram_di;
	
	assign {eep_ram_di[7:0], eep_on, bus_conflicts, map_led, mem_dma, mir_4sc, map_ppu_oe, map_cpu_oe, 
	pwm, map_irq, chr_oe, prg_mem_oe, chr_ce, srm_ce, prg_ce, chr_we, srm_we, prg_we, 
	int_ciram_ce, int_ciram_a10, chr_addr[22:0], prg_addr[22:0], map_ppu_dout[7:0], map_cpu_dout[7:0]} = map_out[`BW_MAP_OUT-1:8];
	
	wire map_irq;
	assign cpu_irq = map_irq & epsm_irq;
	
	wire [`BW_MAP_OUT-1:0]bus = {
	chr_dat[7:0], prg_dat[7:0], m3, os_act, fds_sw, sys_rst, map_rst, 
	clk, ppu_we, ppu_oe, ppu_addr[13:0], ppu_dat[7:0], m2, cpu_rw, 
	{!cpu_ce, cpu_addr[14:0]}, cpu_dat_int[7:0]};
	
	wire [7:0]cpu_dat_int = bus_conf_act ? (cpu_dat[7:0] & prg_dat[7:0]) : cpu_dat[7:0];
	wire bus_conf_act = bus_conflicts & !prg_ce & !cpu_rw;
	
	wire m3 = m[8] & m2;//m2 with delayed rising edge. required for mem async write operations and some other stuff
	reg [8:0]m;	
	always @(negedge clk)m[8:0] <= {m[7:0], m2};
//**************************************************************************************** data bus driver
	wire apu_area = {!cpu_ce, cpu_addr[14:5], 5'd0} == 16'h4000;
	wire cart_space = (!cpu_ce | cpu_addr[14]) & !apu_area;
	
	//cpu
	assign cpu_dat[7:0] = 
	cpu_dir == 0 ? 8'hzz : 
	io_oe_cpu ? io_dout_cpu[7:0] : 
	ss_oe_cpu ? ss_do[7:0] :
	map_cpu_oe ? map_cpu_dout[7:0] : 
	gg_oe ? gg_do[7:0] : 
	(!prg_ce | srm_ce) & !prg_oe ? prg_dat[7:0] : 
	{!cpu_ce, cpu_addr[14:8]};//open bus
	
	assign cpu_dir = cart_space & cpu_rw & m3 ? 1 : 0;// cpu_bus_oe;
	assign cpu_ex  = mem_dma ? 1 : 0;
	
	
	//ppu
	assign ppu_dat[7:0] = 
	ppu_dir == 0 ? 8'hzz : 
	mem_dma ? 8'h00 : 
	map_ppu_oe ? map_ppu_dout[7:0] :
	ppu_iram_oe ? ppu_ram_do[7:0] : 
	!chr_ce & !chr_oe ? chr_dat[7:0] : 
	8'hff;
	
	assign ppu_dir = !ppu_oe & ppu_ciram_ce ? 1 : 0;
	assign ppu_ex = 0;//mem_dma ? 1 : 0;
//**************************************************************************************** memory driver
	assign prg_lb = prg_addr[22];
	assign prg_ub = !prg_addr[22];
	assign chr_lb = chr_addr[22];
	assign chr_ub = !chr_addr[22];
	
	
	assign prg_dat[7:0] = 
	!prg_oe ? 8'hzz :
	eep_on  ? eep_ram_di[7:0] : 
	mem_dma | map_cpu_oe ? map_cpu_dout[7:0] : cpu_dat[7:0];
	
	assign chr_dat[7:0] = 
	!chr_oe ? 8'hzz : 
	mem_dma | map_ppu_oe ? map_ppu_dout[7:0] : ppu_dat[7:0];

	
	assign prg_oe = prg_mem_oe & !bus_conf_act;
	assign srm_oe = prg_mem_oe;
//**************************************************************************************** vram driver
	assign ppu_ciram_a10 = !mir_4sc_act ? int_ciram_a10 : ppu_addr[10];
	assign ppu_ciram_ce =  !mir_4sc_act ? int_ciram_ce  : !(!int_ciram_ce & ppu_addr[11] == 0);
	wire mir_4sc_act = cfg_mir_4 & mir_4sc;
	wire ppu_iram_ce = mir_4sc_act & !int_ciram_ce & ppu_addr[11] == 1;
	wire ppu_iram_oe = ppu_iram_ce & !ppu_oe;
	wire [7:0]ppu_ram_do;
	ppu_ram ppu_ram_inst(ppu_addr[10:0], ppu_dat, ppu_ram_do, ppu_iram_ce, !ppu_oe, !ppu_we, clk);	
//**************************************************************************************** system mappers
	wire [`BW_MAP_OUT-1:0]map_out = 
	dma_req ? map_out_dma : 
	os_act ? map_out_255 : 
	map_out_hub;	
	
	
	wire [7:0]pi_dat_os;
	
	wire [`BW_MAP_OUT-1:0]map_out_255;
	map_255 m255(map_out_255, bus, sys_cfg, ss_ctrl);
	
	wire [`BW_MAP_OUT-1:0]map_out_dma;
	wire [7:0]pi_dat_dma;
	wire dma_req;
	map_dma dma_inst(map_out_dma, bus, pi_bus, pi_dat_dma, dma_req);
	
	
	wire [`BW_MAP_OUT-1:0]map_out_hub;
	map_hub hub_inst(sys_cfg, bus, map_out_hub, ss_ctrl);
//**************************************************************************************** reset controls		
	wire map_rst = map_idx == 255 | !ctrl_unlock | map_rst_req;
	wire os_act = map_rst | ss_act;
	wire sys_rst;
	sys_rst_ctrl sys_rst_inst(m2, clk, sys_rst);// cpu reset detection

	wire map_rst_ack = map_idx == 255;
	wire map_rst_req;
	map_rst_ctrl map_rst_inst(map_rst_req, sys_rst, map_rst_ack, ctrl_rst_delay, clk);//mapper reset control
//**************************************************************************************** gg	
	wire [7:0]gg_do;
	wire gg_oe;
	
`ifndef GG_OFF	
	gg gg_inst
	(
		.bus(bus), 
		.sys_cfg(sys_cfg),
		.pi_bus(pi_bus), 
		.gg_do(gg_do), 
		.gg_oe(gg_oe)
	);
`endif	

//**************************************************************************************** save state controller	
	wire [`BW_SS_CTRL-1:0]ss_ctrl;
	wire ss_oe_cpu, ss_oe_pi, ss_act;
	wire [7:0]ss_do;
	wire [7:0]ss_rdat = map_out_hub[7:0];

`ifndef SS_OFF		
	sst_controller ss_inst(
		.bus(bus),
		.pi_bus(pi_bus),
		.sys_cfg(sys_cfg),
		.ss_ctrl(ss_ctrl),
		.ss_di(ss_rdat),
		.ss_do(ss_do),
		.ss_oe_cpu(ss_oe_cpu),
		.ss_oe_pi(ss_oe_pi),
		.ss_act(ss_act)
	);
`endif
//**************************************************************************************** base io
	wire io_oe_cpu, io_oe_pi;
	wire [7:0]io_dout_cpu;
	wire [7:0]io_dout_pi;
	wire [`BW_SYS_CFG-1:0]sys_cfg;
	
	
	base_io io_inst(
		.bus(bus),
		.pi_bus(pi_bus),
		.sys_cfg(sys_cfg),
		.dout_pi(io_dout_pi),
		.dout_cpu(io_dout_cpu),
		.io_oe_pi(io_oe_pi),
		.io_oe_cpu(io_oe_cpu),
		.pi_fifo_rxf(fifo_rxf),
		.mcu_busy(mcu_busy)
	);

	
	
	wire [7:0]pi_di = 
	dma_req ? pi_dat_dma : 
	ss_oe_pi ? ss_do : 
	io_oe_pi ? io_dout_pi :
	8'hff;
	
	wire [`BW_PI_BUS-1:0]pi_bus;
	
	pi_interface pi_inst(
		.miso(spi_miso), 
		.mosi(spi_mosi), 
		.ss(spi_ss), 
		.clk(spi_clk), 
		.din(pi_di), 
		.pi_bus(pi_bus)
	);
	
	wire epsm_irq;
    epsmirqs epsmirqs_inst(
        .ext_CPU_ADDR(cpu_addr),
        .CPU_DATA(cpu_dat),
        .CPU_ROMSEL(cpu_ce),
        .CPU_RW(cpu_rw),
        .M2(m2),
        .clk(clk),
        .CPU_IRQ(epsm_irq),
    );
	
endmodule


//*********************************************************************************
//*********************************************************************************
//*********************************************************************************

module sys_rst_ctrl
(m2, clk, rst);
	input m2, clk;
	output rst;
	
	assign rst = ctr[6];
	
	reg [1:0]m2_st;
	reg [6:0]ctr;
	
	always @(negedge clk)
	begin
		
		m2_st[1:0] <= {m2_st[0], m2};
		
		if(m2_st[0] != m2_st[1])ctr <= 0;
			else
		if(!rst)ctr <= ctr + 1;
	
	end
	
endmodule
//*********************************************************************************
module map_rst_ctrl
(rst_req, sys_rst, rst_ack, rst_delay, clk);
	
	output reg rst_req;
	input sys_rst, rst_ack, rst_delay, clk;
	
	parameter DELAY_SIZE = 25;
	
	reg rst_st;
	reg rst_ack_st;
	reg [DELAY_SIZE:0]delay;
	
	wire rst_act = rst_st & (delay[DELAY_SIZE:DELAY_SIZE-1] == 2'b11 | !rst_delay);
	
	always @(negedge clk)
	begin
	
		rst_ack_st <= rst_ack;
		rst_st <= sys_rst;
		
		if(rst_act)rst_req <= 1;
			else
		if(rst_ack_st)rst_req <= 0;
		
		
		if(rst_st == 0)delay <= 0;
		if(rst_st == 1 & !rst_act)delay <= delay + 1;	
	end
	
endmodule
//*********************************************************************************
module ppu_ram
(addr, din, dout, ce, oe, we, clk);

	input [11:0]addr;
	input [7:0]din;
	output reg [7:0]dout;
	input ce, oe, we, clk;
	
	
	reg [7:0]ram[4096];
	wire we_act = we & ce & !oe;
	
	
	always @(negedge clk)
	begin
		
		
		if(we_act)ram[addr][7:0] <= din[7:0];
			else
		dout[7:0] <= ram[addr][7:0];
		
		
	end

endmodule

module epsmirqs
  (
   input [14:0] ext_CPU_ADDR,
   input [7:0]  CPU_DATA,
   input        CPU_ROMSEL,
   input        CPU_RW, M2,
   input        clk, 
   output       CPU_IRQ
   );

   reg [9:0]    TimerA_prescaler;
   reg [3:0]    TimerB_prescaler;
   reg [9:0]    TimerA;
   reg [7:0]    TimerB;
   reg [9:0]    TimerAperiod;
   reg [7:0]    TimerBperiod;
   reg [7:0]    EPSM_ADDR;
   reg TimerAIrqUnmask, TimerBIrqUnmask,
       TimerAIrqPermit, TimerBIrqPermit,
       TimerAEnable, TimerBEnable;
   reg TimerAIrq, TimerBIrq;
   reg TimerAIrqReqL, TimerAIrqReqR, // for crossing clock domains
       TimerBIrqReqL, TimerBIrqReqR;

   wire [16:0] CPU_ADDR;
   
   assign CPU_ADDR = {CPU_RW, !CPU_ROMSEL, ext_CPU_ADDR};

   assign CPU_IRQ = !(TimerAIrq & TimerAIrqUnmask) &
	 !(TimerBIrq & TimerBIrqUnmask);
   
   always @(negedge M2) begin
      if (CPU_ADDR == 17'h0401C ||
          CPU_ADDR == 17'h0401E) begin
         EPSM_ADDR <= CPU_DATA;
      end else if (CPU_ADDR == 17'h0401D) begin
         case (EPSM_ADDR)
           8'h24: TimerAperiod[9:2] <= CPU_DATA;
           8'h25: TimerAperiod[1:0] <= CPU_DATA[1:0];
           8'h26: TimerBperiod <= CPU_DATA;
           8'h29:
             { TimerBIrqUnmask,
               TimerAIrqUnmask } <= CPU_DATA[1:0];
           8'h27:
              { TimerBIrqPermit,
                TimerAIrqPermit,
                TimerBEnable,
                TimerAEnable } <= CPU_DATA[3:0];
         endcase // case (EPSM_ADDR)
      end // if (CPU_ADDR == 17'h0401D)
   end // always @ (negedge M2)

   always @(negedge M2) begin
      if (TimerAIrqReqL != TimerAIrqReqR) begin 
         TimerAIrq <= 1;
         TimerAIrqReqR <= TimerAIrqReqL;
      end else if (CPU_ADDR == 17'h0401D && EPSM_ADDR == 8'h27) begin
         if (CPU_DATA[4] == 1)
           TimerAIrq <= 0;
      end
      if (TimerBIrqReqL != TimerBIrqReqR) begin 
         TimerBIrq <= 1;
         TimerBIrqReqR <= TimerBIrqReqL;
      end else if (CPU_ADDR == 17'h0401D && EPSM_ADDR == 8'h27) begin
         if (CPU_DATA[5] == 1)
           TimerBIrq <= 0;
      end
   end // always @ (negedge M2)

   // clk is the 50MHz clock source on the EDN8Pro
   always @(posedge clk) begin
      if (TimerA_prescaler == 10'd0) begin
        TimerA_prescaler <= 1024-900; // 450 = 50MHz / 8MHz * 72

         if (TimerAEnable) begin
           if (TimerA == 10'd0) begin
              TimerA <= TimerAperiod;
           end else if (TimerA == 10'h3FF) begin
              if (TimerAIrqPermit) TimerAIrqReqL <= !TimerAIrqReqL;
              TimerA <= 10'd0;
           end else TimerA <= TimerA + 1;
         end else TimerA <= 0;

         TimerB_prescaler <= TimerB_prescaler + 1; // 16

         if (TimerB_prescaler == 4'd0) begin 
            if (TimerBEnable) begin
               if (TimerB == 8'd0) begin
                  TimerB <= TimerBperiod;
               end else if (TimerB == 8'hFF) begin
                  if (TimerBIrqPermit) TimerBIrqReqL <= !TimerBIrqReqL;
                  TimerB <= 8'd0;
               end else TimerB <= TimerB + 1;
            end else TimerB <= 0;
         end

      end else TimerA_prescaler <= TimerA_prescaler + 1;
   end

endmodule // epsmirqs

