`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.12.2025 15:30:00
// Design Name: 
// Module Name: top
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


module baud_rate_generator #(
    parameter BAUD_DIV_WIDTH = 16,
    parameter BAUD_DIV = 434 
)(
    input logic clock,
    input logic reset,
    output logic baud_rate_tick
);

    logic [BAUD_DIV_WIDTH-1:0] cnt;
    
    always_ff @(posedge clock) begin
        if (reset || (cnt == BAUD_DIV))
            cnt <= {BAUD_DIV_WIDTH{1'b0}};
        else
            cnt <= cnt + 1;
    end
    
    assign baud_rate_tick = (cnt == BAUD_DIV);
endmodule   

module uart_system_receiver #(  //BUG WITH RX_DONE
    parameter DBITS = 8,
    parameter SBITS = 1
)(
    input logic clock,
    input logic reset,
    input logic rx,
    output logic [DBITS-1:0] data_out,
    output logic rx_done
    /*output logic ttest_reciever,
    output logic ttest_reciever1,
    output logic ttest_reciever2,
    output logic ttest_reciever3*/
);

    logic sample_tick;
    logic local_reset;
    assign local_reset = reset;
    
    baud_rate_generator #(
        .BAUD_DIV_WIDTH(16),
        .BAUD_DIV(326)
    ) brg_inst (
        .clock(clock),
        .reset(local_reset),
        .baud_rate_tick(sample_tick)
    );

    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_r;
    state_uart_r state, next_state;
    
    logic [DBITS -1:0] shift_reg, shift_reg_next;
    logic [3:0] s_cnt, s_cnt_next;
    logic [3:0] n_cnt, n_cnt_next;
    logic rx_done_next;
    
    localparam STOP_TICKS = SBITS * 16;
    
    always_ff @(posedge clock) begin
        if(reset) begin
            state <= IDLE;
            shift_reg <= {DBITS{1'b0}};
            s_cnt <= 4'd0;
            n_cnt <= 4'd0;
            rx_done <= 1'b0;
        end else begin
            state <= next_state;
            shift_reg <= shift_reg_next;
            s_cnt <= s_cnt_next;
            n_cnt <= n_cnt_next;
            rx_done <= rx_done_next;
        end
    end

    always_comb begin
        next_state = state;
        rx_done_next = 1'b0;
        shift_reg_next = shift_reg;
        s_cnt_next = s_cnt;
        n_cnt_next = n_cnt;
        
        case (state)
            IDLE: begin
                if(rx == 1'b0) begin
                    next_state = START;
                    s_cnt_next = 0;
                end
            end
            
            START: begin
                if(sample_tick) begin
                    if(s_cnt == 4'd7) begin
                        next_state = DATA;
                        s_cnt_next = 0;
                        n_cnt_next = 0;
                    end else begin
                        s_cnt_next = s_cnt + 1;
                    end
                end
            end
            
            DATA: begin
                if(sample_tick) begin
                    if(s_cnt == 4'd15) begin
                        s_cnt_next = 0;
                        shift_reg_next = {rx, shift_reg[DBITS-1:1]};
                        if (n_cnt == DBITS -1) begin
                            next_state = STOP;
                        end else begin
                            n_cnt_next = n_cnt + 1;
                        end
                    end else begin
                        s_cnt_next = s_cnt + 1;
                    end
                end
            end
            
            STOP: begin // we never reach this state?
                if (sample_tick) begin
                    if(s_cnt == STOP_TICKS-1) begin
                        rx_done_next = 1'b1;
                        next_state = IDLE;
                    end else begin
                        s_cnt_next = s_cnt +1;
                    end
                end
            end                 
        endcase
    end

    /*asssign ttest_reciever = (state == IDLE);
    asssign ttest_reciever1 = (state == START);
    asssign ttest_reciever2 = (state == DATA);
    asssign ttest_reciever3 = (state == STOP);*/

    assign data_out = shift_reg;

endmodule

module uart_system_transmitter (
    input logic clock,
    input logic reset,
    input logic [7:0] wr_data,
    input logic wr_uart,
    output logic tx_done,
    output logic tx
);

    logic baud_rate_tick;
    logic baud_rst;
    logic local_reset;
    
    assign local_reset = reset | baud_rst;
    
    baud_rate_generator #(
        .BAUD_DIV_WIDTH(16),
        .BAUD_DIV(5209)
    ) brg_inst (
        .clock(clock),
        .reset(local_reset),
        .baud_rate_tick(baud_rate_tick)
    );
    
    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_t;
    state_uart_t state, next_state;
    
    logic [7:0] tx_data;
    logic [2:0] data_cnt;
    logic start_trans, end_data, end_start, end_stop;
    
    always_ff @(posedge clock) begin : tx_data_reg
        if (reset)
            tx_data <= 8'd0;
        else if (wr_uart && (state == IDLE))
            tx_data <= wr_data;
    end
    
    always_comb begin : tx_line
        // default
        tx = 1'b1;
        case (state)
            START: tx = 1'b0;
            DATA: begin
                case (data_cnt)
                    3'd0: tx = tx_data[0];
                    3'd1: tx = tx_data[1];
                    3'd2: tx = tx_data[2];
                    3'd3: tx = tx_data[3];
                    3'd4: tx = tx_data[4];
                    3'd5: tx = tx_data[5];
                    3'd6: tx = tx_data[6];
                    3'd7: tx = tx_data[7];
                    default: tx = 1'b1;
                endcase
            end
            IDLE, STOP:  tx = 1'b1;
        endcase
    end

    always_ff @(posedge clock)begin : data_counter
        if (reset || end_data)
            data_cnt <= 3'd0;
        else if(state == DATA && baud_rate_tick)
            data_cnt <= data_cnt +1'b1;
    end
    
    always_ff @(posedge clock) begin : advance
        if (reset)
            state <= IDLE;
        else 
            state <= next_state;
    end
    
    always_comb begin: ust_signals
        start_trans = (state == IDLE) && wr_uart;
        end_start = (state == START) && baud_rate_tick;
        end_stop = (state == STOP) && baud_rate_tick;
        end_data = (state == DATA) && (data_cnt == 3'd7) && baud_rate_tick;
    end
    
    always_comb begin: ust_transition
        // priority transitions (avoid complex ternary expressions)
        next_state = state;
        if (start_trans)
            next_state = START;
        else if (end_start)
            next_state = DATA;
        else if (end_data)
            next_state = STOP;
        else if (end_stop)
            next_state = IDLE;
    end
    
    
    assign tx_done = end_stop;
    assign baud_rst = (state == IDLE);
    
endmodule

module shift_reg_with_par_read #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 6
)(
    input  logic clk,
    input  logic rst,
    input  logic wr_en,
    input  logic rd_en,
    input  logic [DATA_WIDTH-1:0] wr_data,
    output logic [DATA_WIDTH*DEPTH-1:0] rd_data
);

    // Simple shift-register array with parallel read output.
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= {DATA_WIDTH{1'b0}};
        end else if (wr_en) begin
            // shift towards higher indices, new data at mem[0]
            for (i = DEPTH-1; i > 0; i = i - 1)
                mem[i] <= mem[i-1];
            mem[0] <= wr_data;
        end
    end

    // parallel read output: most-recent at low bits (mem[0])
    // Use generate + continuous assigns (avoids procedural part-select issues)
    genvar gi, bj;
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : rd_assign
            for (bj = 0; bj < DATA_WIDTH; bj = bj + 1) begin : bit_assign
                assign rd_data[gi*DATA_WIDTH + bj] = mem[gi][bj];
            end
        end
    endgenerate

endmodule

module rgb_controller (
    input logic clock,
    input logic reset,
    input logic [5:0] SW,
    output logic [2:0] RGB,
    output logic cr1,
    output logic cr2,
    output logic cg1,
    output logic cg2,
    output logic cb1,
    output logic cb2
);

    logic [1:0] red, green, blue;
    logic [2:0] pwm_count;
    
    assign red = SW[1:0];
    assign green = SW[3:2];
    assign blue = SW[5:4];
    
    always_ff @(posedge clock) begin
        if(reset) begin
            pwm_count <= 3'b0;
        end else begin
            pwm_count <= pwm_count + 1;
        end
    end
    
    assign RGB[2] = (pwm_count < red);
    assign RGB[1] = (pwm_count < green);
    assign RGB[0] = (pwm_count < blue);
    
    /*assign cr1 = SW[0];
    assign cr2 = SW[1];
    assign cg1 = SW[2];
    assign cg2 = SW[3];
    assign cb1 = SW[4];
    assign cb2 = SW[5];*/
    

endmodule

module top (
    input  logic        clk,        // System clock
    input  logic        rst,      // Active-high reset
    input  logic        uart_rx,    // UART receive line
    output logic        uart_tx,     // UART transmit line
    output logic        PWM_red,
    output logic        PWM_green,
    output logic        PWM_blue,
    output logic        done
    /*output logic        r1,
    output logic        r2,
    output logic        g1,
    output logic        g2,
    output logic        b1,
    output logic        b2*/
    /*output logic        test_reciever,
    output logic        test_reciever1,
    output logic        test_reciever2,
    output logic        test_reciever3*/
);

    // Internal signals
    logic [7:0] rx_data;            // Data received from UART
    logic       tx_done, tx_done_next;           // Ready signal for UART transmitter
    logic       tx_start, tx_start_next;           // Start signal for UART transmitter
    logic [7:0] tx_data;            // Data to be transmitted via UART

    // To uart system receiver
    logic rx_read, rx_read_next; 
    logic rx_empty;
    logic rx_valid; 
    logic rx_done; // Data received from UART


    logic done_next;

    // UART Receiver instance
    uart_system_receiver #(
        .DBITS (8),
        .SBITS (1)
    ) uart_receiver (
        .clock     (clk),
        .reset     (rst),
        .rx        (uart_rx),
        .data_out  (rx_data),
        .rx_done (rx_done)
        /*.ttest_reciever (test_reciever),
        .ttest_reciever1 (test_reciever1),
        .ttest_reciever2 (test_reciever2),
        .ttest_reciever3 (test_reciever3)*/
    );

    // UART Transmitter instance
    uart_system_transmitter uart_transmitter (
        .clock    (clk),
        .reset    (rst),
        .wr_data  (tx_data),
        .wr_uart  (tx_start),
        .tx_done  (tx_done),
        .tx       (uart_tx)
    );

    // Shift register with parallel read instance
    logic rd_en, rd_en_next;
    logic [47:0] token;
    logic shift_reg_reset;
    assign shift_reg_reset = rst | done;

    shift_reg_with_par_read #(
        .DATA_WIDTH (8),   
        .DEPTH      (6)
    ) u_shift_reg_with_par_read (
        .clk        (clk),
        .rst        (shift_reg_reset),
        .wr_en      (rx_done),
        .rd_en      (rd_en),
        .wr_data    (rx_data),
        .rd_data    (token)
    );


     //Control logic for RGB
    // on ctrl_valid signal we update the ctrl register 
    // ctrl register holds the 6 bit control signal for RGB controller 
    // extracted from the token
    logic [5:0] ctrl;
    logic ctrl_valid, ctrl_valid_next;

       // Instantiate the RGB controller

    logic rgb_controller_reset;
    logic rgb_controller_start;
    assign rgb_controller_reset = rst | rgb_controller_start;
    logic [5:0] ctrl_reg, ctrl_next;
    
    rgb_controller rgb_controller_inst (
        .clock  (clk),
        .reset  (rgb_controller_reset),
        .SW     (ctrl_reg),
        .RGB    ({PWM_red, PWM_green, PWM_blue})
        /*.cr1    (r1),
        .cr2    (r2),
        .cg1    (g1),
        .cg2    (g2),
        .cb1    (b1),
        .cb2    (b2)*/
    );
 


    // Message bytes to be transmitted
    // "Ready\n" and "Control\n"
    logic [7:0] message_byte [0:17];
    initial begin
        message_byte[0]  = "R";
        message_byte[1]  = "e";
        message_byte[2]  = "a";
        message_byte[3]  = "d";
        message_byte[4]  = "y";
        message_byte[5]  = "\n";
        message_byte[6] = 13;
        message_byte[7] = 8'h0; // Null terminator
        message_byte[8]  = "C";
        message_byte[9]  = "o";
        message_byte[10]  = "n";
        message_byte[11]  = "t";
        message_byte[12] = "r";
        message_byte[13] = "o";
        message_byte[14] = "l";
        message_byte[15] = "\n";
        message_byte[16] = 13;
        message_byte[17] = 8'h0; // Null terminator
    end


 
    // control logic for UART transmission


    // define the states
    typedef enum logic [1:0] { // binary encoding
        PRINT1,
        AWAIT,
        CONTROL,
        PRINT2
    } state_echo;

    state_echo state, next_state;
    logic [4:0] tx_index, tx_index_next;

    // internal control registers
    logic tx_start_reg; // pulse to start a tx
    logic tx_sent, tx_sent_next;       // indicates we've pulsed tx_start for current char
    logic rd_en_reg;
    logic ctrl_valid_reg;
    logic done_reg;
    // ctrl register and next value (6-bit RGB control)
    //logic [5:0] ctrl_reg, ctrl_next;
    logic [47:0] token_latched;
    
    assign test_light4 = rx_done;
    
    logic [3:0] counter_rx;
    always_ff @(posedge clk) begin
        if (rx_done) begin
            if (rst)
                counter_rx <= 0;
            else if (counter_rx == 6)
                counter_rx <= 0;
            else 
                counter_rx <= counter_rx + 1;
        end
    end
    
    logic stop_flag;
    assign stop_flag = (counter_rx == 6 && rx_done) ? 1'b1 : 1'b0;
    
    // drive outputs
    assign tx_start = tx_start_reg;
    assign rd_en = rd_en_reg;
    assign ctrl_valid = ctrl_valid_reg;
    assign done = done_reg;

    // tx_data is the current message byte indexed by tx_index
    assign tx_data = message_byte[tx_index];

    // sequential registers
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= PRINT1;
            tx_index <= 4'd0;
            tx_start_reg <= 1'b0;
            tx_sent <= 1'b0;
            rd_en_reg <= 1'b0;
            ctrl_valid_reg <= 1'b0;
            done_reg <= 1'b0;
            token_latched <= {48{1'b0}};
            ctrl_reg <= 6'd0;
        end else begin
            state <= next_state;
            tx_index <= tx_index_next;
            tx_start_reg <= tx_start_next;
            tx_sent <= tx_sent_next;
            rd_en_reg <= rd_en_next;
            ctrl_valid_reg <= ctrl_valid_next;
            done_reg <= done_next;
            // latch token when rd_en asserted
            if (rd_en_next)
                token_latched <= token;
            // update ctrl register
            ctrl_reg <= ctrl_next;
        end
    end

    // combinational next-state and outputs
    always_comb begin
        // defaults: hold values
        next_state = state;
        tx_index_next = tx_index;
        tx_start_next = 1'b0;
        tx_sent_next = tx_sent;
        rd_en_next = 1'b0;
        ctrl_valid_next = 1'b0;
        done_next = 1'b0;
        // default ctrl_next holds previous value
        ctrl_next = ctrl_reg;

        case (state)
            // PRINT1: send "Ready\n" bytes [0..5]
            PRINT1: begin
                if (!tx_sent) begin
                    // start transmitting current byte
                    tx_start_next = 1'b1;
                    tx_sent_next = 1'b1;
                end else begin
                    // wait for tx_done to advance
                    if (tx_done) begin
                        tx_sent_next = 1'b0;
                        if (tx_index == 4'd6) begin
                            // finished printing "Ready\n"
                            done_next = 1'b1; // pulse done to reset shift reg
                            //tx_index_next = 4'd0;
                            tx_index_next = tx_index + 1;
                            next_state = AWAIT;
                        end else begin
                            tx_index_next = tx_index + 1;
                        end
                    end
                end
            end

            // AWAIT: wait for newline (stop_flag). When seen, assert rd_en one cycle and go to CONTROL
            AWAIT: begin
                if (stop_flag) begin
                    rd_en_next = 1'b1;
                    next_state = CONTROL;
                end
            end

            // CONTROL: one cycle - parse token_latched and update ctrl
            CONTROL: begin
                // extract characters according to shift order (mem[0] = last received)
                logic [7:0] R_char, G_char, B_char;
                logic [1:0] R_val, G_val, B_val;
                // explicit ranges (avoid +: part-select in procedural block)
                R_char = token_latched[39:32];
                G_char = token_latched[23:16];
                B_char = token_latched[7:0];

                // convert ascii digits '0'..'3' to 2-bit values, clamp otherwise
                R_val = (R_char >= "0" && R_char <= "3") ? (R_char - "0") : 2'd0;
                G_val = (G_char >= "0" && G_char <= "3") ? (G_char - "0") : 2'd0;
                B_val = (B_char >= "0" && B_char <= "3") ? (B_char - "0") : 2'd0;

                // ctrl mapping: [5:4]=R, [3:2]=G, [1:0]=B
                ctrl_next = {B_val, G_val, R_val};
                ctrl_valid_next = 1'b1;

                // prepare for PRINT2
                tx_index_next = 4'd8; // starting index for "Control\n"
                tx_sent_next = 1'b0;
                next_state = PRINT2;
            end

            // PRINT2: send "Control\n" bytes [7..14]
            PRINT2: begin
                if (!tx_sent) begin
                    tx_start_next = 1'b1;
                    tx_sent_next = 1'b1;
                end else begin
                    if (tx_done) begin
                        tx_sent_next = 1'b0;
                        if (tx_index == 5'd17) begin
                            tx_index_next = 4'd0;
                            next_state = PRINT1;
                        end else begin
                            tx_index_next = tx_index + 1;
                        end
                    end
                end
            end

            default: begin
                next_state = PRINT1;
            end
        endcase
    end

    /*assign test_light = (state == PRINT1);
    assign test_light1 = (state == AWAIT);
    assign test_light2 = (state == CONTROL);
    assign test_light3 = (state == PRINT2);
    assign test_uart_rx = uart_rx;*/
    
    /**always_ff @(posedge clk) begin  
        if (uart_rx == 0)
            test_uart_rx <= 1;
        if (state == CONTROL)
            test_light2 <= 1;
        if (state == PRINT2)
            test_light3 <= 1;
        if (rst) begin
            test_uart_rx <= 0;
            test_light2 <= 0;
            test_light3 <= 0;
        end
    end*/


endmodule
