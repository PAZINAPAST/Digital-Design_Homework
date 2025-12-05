`timescale 1ns/1ps

module top_tb;
    // Testbench signals
    logic        clk;
    logic        rst;
    logic        start;
    logic        uart_rx;
    logic        uart_tx;
    logic        done;
    logic        PWM_red;
    logic        PWM_green;
    logic        PWM_blue;
    
    // Parameters for UART timing
    // Assuming 100MHz clock and 9600 baud rate
    localparam CLOCK_PERIOD = 10;  // 10ns, 100MHz
    localparam BIT_PERIOD = 52083; // ~19200 baud in ns
    
    // DUT instantiation
    top dut (
        .clk(clk),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .done(done),
        .PWM_red(PWM_red),
        .PWM_green(PWM_green),
        .PWM_blue(PWM_blue)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLOCK_PERIOD/2) clk = ~clk;
    end
    
    // Task to send a byte over UART RX line
    task send_byte(input logic [7:0] data);
        // Start bit
        uart_rx = 0;
        #BIT_PERIOD;
        
        // Data bits (LSB first)
        for (int i = 0; i < 8; i++) begin
            uart_rx = data[i];
            #BIT_PERIOD;
        end
        
        // Stop bit
        uart_rx = 1;
        #BIT_PERIOD;
    endtask
    
    // Task to read a byte from UART TX line
    task receive_byte(output logic [7:0] data);
        // Wait for start bit
        @(negedge uart_tx);
        #(BIT_PERIOD/2); // Sample in middle of bit
        
        // Verify start bit
        if (uart_tx != 0) $display("Error: Invalid start bit");
        #BIT_PERIOD;
        
        // Read data bits (LSB first)
        for (int i = 0; i < 8; i++) begin
            #(BIT_PERIOD/2);
            data[i] = uart_tx;
            #(BIT_PERIOD/2);
        end
        
        // Check stop bit
        if (uart_tx != 1) $display("Error: Invalid stop bit");
        #BIT_PERIOD;
    endtask
    
    // Test sequence
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars;
        // Initialize signals
        uart_rx = 1; // Idle state for UART
        rst = 0;     // Active-low reset
        start = 0;
        
        // Apply reset
        #(CLOCK_PERIOD*5);
        rst = 1;
        #(CLOCK_PERIOD*10);
        
        // Release reset
        rst = 0;
        // Start the echo process
        start = 1;
        #(CLOCK_PERIOD*2);
        start = 0;
        
        // Send test data
        $display("Printing the data:");
        wait(done == 1);
        $display("Test 1: Multiple Characters");
        send_byte(8'h52); // 'R'
        send_byte(8'h31); // '1'
        send_byte(8'h47); // 'G'
        send_byte(8'h32); // '2'
        send_byte(8'h42); // 'B'
        send_byte(8'h33); // '3'
        send_byte(8'h0A); // New line to signal end
        send_byte(8'h52); // 'R'
        send_byte(8'h30); // '0'
        send_byte(8'h47); // 'G'
        send_byte(8'h32); // '2'
        send_byte(8'h42); // 'B'
        send_byte(8'h30); // '0'
        send_byte(8'h0A); // New line to signal end
        send_byte(8'h0A); // New line to signal end
        #(CLOCK_PERIOD*30);
        wait(done);
        $display("Test Completed");

        $finish;
        // End simulation
       
    end

    // End simulation
    // initial begin
    //     wait(done);
    //     $display("Test Completed");
    //     #(CLOCK_PERIOD*30);
    //     wait(done);
    //     $display("Test Completed");
    //     #(CLOCK_PERIOD*30);
    //     $display("Simulation Done");
    //     $finish;
    // end
    
endmodule