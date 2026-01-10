`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.01.2026 13:59:21
// Design Name: 
// Module Name: spi_apb_wrapper
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module spi_apb_wrapper(
    input logic PCLK,
    input logic PRESETn,
    input logic PSEL,
    input logic PENABLE,
    input logic PWRITE,
    input logic [7:0] PADDR,
    input logic [31:0] PWDATA,
    output logic [31:0] PRDATA,
    output logic PREADY,
    
    //SPI pini
    input logic spi_miso,
    output logic spi_mosi,
    output logic spi_clk,
    output logic spi_cs_n
    
);
    // registri apb wrapperja
    logic start_reg;
    logic [7:0] txdata_reg;
    logic [15:0] limit_reg;
    
    // SPI outputi
    logic done;
    logic [7:0] data_out;
    logic [3:0] symbol_counter;
    
    // instanciranje SPI
    spiModule spi_inst (
        .clk(PCLK),
        .rst(~PRESETn),
        .data_in(txdata_reg),
        .data_out(data_out),
        .spi_miso(spi_miso),
        .start(start_reg),
        .limit(limit_reg),
        .spi_clk(spi_clk),
        .spi_mosi(spi_mosi),
        .done(done)
    );
    
    logic cs_active;
    
    // chip select aktiven med prenosom
    assign spi_cs_n = ~cs_active;
    //apb ready
    assign PREADY = 1'b1;
    
    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) 
            cs_active <= 1'b0;
        else if (start_reg)
            cs_active <= 1'b1;
        else if (done)
            cs_active <= 1'b0;
    end
    
    //Write logic
    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            start_reg <= 1'b0;
            txdata_reg <= 8'd0;
            limit_reg <= 16'd4;// osnovna spi hitrost
        end else if (PSEL && PENABLE && PWRITE) begin
            case (PADDR)
                8'h00: begin
                    start_reg <= PWDATA[0]; // bit 0 = start
                    limit_reg <= PWDATA[31:16];
                end
                8'h04: txdata_reg <= PWDATA[7:0];
            endcase
        end else begin
            start_reg <= 1'b0;
        end
    end
    
    //ready logika
    always_comb begin
        PRDATA = 32'd0;
        case (PADDR)
            8'h00: PRDATA = {limit_reg, 15'd0, start_reg};
            8'h04: PRDATA = {24'd0, txdata_reg};
            8'h08: PRDATA = {24'd0, data_out};
            8'h0C: PRDATA = {24'd0, 7'd0, done};
        endcase
    end
    
endmodule
