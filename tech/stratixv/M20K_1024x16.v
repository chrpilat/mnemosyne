module M20K_1024x16 (
	A0,
	A1,
	CLK,
	D0,
	D1,
	CE0,
	CE1,
	WE0,
	WE1,
	WEM0,
	WEM1,
	Q0,
	Q1);

	input	[9:0]  A0;
	input	[9:0]  A1;
	input	  CLK;
	input	[15:0]  D0;
	input	[15:0]  D1;
	input	  CE0;
	input	  CE1;
	input	  WE0;
	input	  WE1;
	input	 [15:0] WEM0;
	input	 [15:0] WEM1;
	output	[15:0] Q0;
	output	[15:0] Q1;

	wire rden_a = CE0 & ~WE0;
	wire rden_b = CE1 & ~WE1;
	wire wren_a = CE0 & WE0;
	wire wren_b = CE1 & WE1;

	wire [15:0] sub_wire0;
	wire [15:0] sub_wire1;
	wire [15:0] Q0 = sub_wire0[15:0];
	wire [15:0] Q1 = sub_wire1[15:0];

	altsyncram	altsyncram_component (
				.clock0 (CLK),
				.wren_a (wren_a),
				.address_b (A1),
				.data_b (D1),
				.rden_a (rden_a),
				.wren_b (wren_b),
				.address_a (A0),
				.data_a (D0),
				.rden_b (rden_b),
				.q_a (sub_wire0),
				.q_b (sub_wire1),
				.aclr0 (1'b0),
				.aclr1 (1'b0),
				.addressstall_a (1'b0),
				.addressstall_b (1'b0),
				.byteena_a (1'b1),
				.byteena_b (1'b1),
				.clock1 (1'b1),
				.clocken0 (1'b1),
				.clocken1 (1'b1),
				.clocken2 (1'b1),
				.clocken3 (1'b1),
				.eccstatus ());
	defparam
		altsyncram_component.address_reg_b = "CLOCK0",
		altsyncram_component.clock_enable_input_a = "BYPASS",
		altsyncram_component.clock_enable_input_b = "BYPASS",
		altsyncram_component.clock_enable_output_a = "BYPASS",
		altsyncram_component.clock_enable_output_b = "BYPASS",
		altsyncram_component.indata_reg_b = "CLOCK0",
		altsyncram_component.intended_device_family = "Stratix V",
		altsyncram_component.lpm_type = "altsyncram",
		altsyncram_component.maximum_depth = 1024,
		altsyncram_component.numwords_a = 1024,
		altsyncram_component.numwords_b = 1024,
		altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
		altsyncram_component.outdata_aclr_a = "NONE",
		altsyncram_component.outdata_aclr_b = "NONE",
		altsyncram_component.outdata_reg_a = "UNREGISTERED",
		altsyncram_component.outdata_reg_b = "UNREGISTERED",
		altsyncram_component.power_up_uninitialized = "FALSE",
		altsyncram_component.ram_block_type = "M20K",
		altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
		altsyncram_component.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
		altsyncram_component.read_during_write_mode_port_b = "NEW_DATA_NO_NBE_READ",
		altsyncram_component.widthad_a = 10,
		altsyncram_component.widthad_b = 10,
		altsyncram_component.width_a = 16,
		altsyncram_component.width_b = 16,
		altsyncram_component.width_byteena_a = 1,
		altsyncram_component.width_byteena_b = 1,
		altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0";

endmodule

