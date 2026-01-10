`timescale 1ns/1ps
`include "../APB_master.sv"


module APB_master_tb;

    // Parameters
    parameter DW = 32;  // Data width
    parameter AW = 32;  // Address width
    localparam CW = 1 + DW + AW;  // Command width
    localparam RW = DW;  // Response width

    // Signals
    logic CLK;
    logic RESETn;
    logic [CW-1:0] i_cmd;
    logic i_valid;
    logic [RW-1:0] o_resp;
    logic o_ready;
    logic [AW-1:0] pADDR;
    logic pSELx;
    logic pENABLE;
    logic pWRITE;
    logic [DW-1:0] pWDATA;
    logic [DW-1:0] pRDATA;
    logic pREADY;
    logic [15:0] switch;
    logic [15:0] led;

    // DUT instantiation
    APB_master #(
        .DW(DW),
        .AW(AW)
    ) dut (
        .pCLK(CLK),
        .pRESETn(RESETn),
        .i_cmd(i_cmd),
        .i_valid(i_valid),
        .o_resp(o_resp),
        .o_ready(o_ready),
        .pADDR(pADDR),
        .pSELx(pSELx),
        .pENABLE(pENABLE),
        .pWRITE(pWRITE),
        .pWDATA(pWDATA),
        .pRDATA(pRDATA),
        .pREADY(pREADY)
    );

    // Clock generation
    always #5 CLK = ~CLK;

    // Testbench logic
    initial begin
        $dumpfile("APB_master_tb.vcd");
        $dumpvars(0, APB_master_tb);
        // Initialize signals
        CLK = 1;
        RESETn = 0;
        i_cmd = 0;
        i_valid = 0;
        pRDATA = 0;
        pREADY = 0;

        // Reset the DUT
        #10
        RESETn = 1;
        pREADY = 1; // Simulate peripheral ready

        // Test 1: Write operation
        #10
        i_cmd = {1'b1, 32'hA5A5A5A5, 32'h00000010}; // Write command: pWRITE=1, pWDATA=0xA5A5A5A5, pADDR=0x10
        i_valid = 1;
        #10
        i_valid = 0;        
        #10;
        //pREADY = 0;

        // Test 2: Read operation with stalled response
        #10
        i_cmd = {1'b0, 32'h00000000, 32'h00000020}; // Read command: pWRITE=0, pADDR=0x20
        i_valid = 1;
        #10
        i_valid = 0;
        #40 
        pRDATA = 32'hDEADBEEF; // Simulate read data
        pREADY = 1; // Simulate peripheral ready
        #10;
        i_valid = 0;
        pREADY = 0;

        // Test 3: Back-to-back transactions
        #10;
        i_cmd = {1'b1, 32'h12345678, 32'h00000030}; // Write command
        i_valid = 1;
        #10;
        pREADY = 1;
        #10;
        i_cmd = {1'b0, 32'h00000000, 32'h00000040}; // Read command
        pRDATA = 32'hCAFEBABE;
        #10;
        //i_valid = 0;
        pREADY = 0;

        // Finish simulation
        #20 $finish;
    end

   

endmodule


