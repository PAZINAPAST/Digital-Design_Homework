module baud_rate_generator #(
    parameter BAUD_DIV_WIDTH = 16,
    parameter BAUD_DIV = 434 
)(
    input logic clock,
    input logic reset,
    output logic baud_tick
);

    logic [BAUD_DIV_WIDTH-1:0] cnt;
    
    always_ff @(posedge clock) begin
        if(reset || (cnt == BAUD_DIV))
            cnt <= '0;
        else
            cnt <= cnt +1'b1;
    end
    
    assign baud_rate_tick = (cnt == BAUD_DIV);
endmodule   

module uart_system_receiver #(
    parameter DBITS = 8,
    parameter SBITS = 1
)(
    input logic clock,
    input logic reset,
    input logic rx,
    output logic [7:0] data_out,
    output logic rx_done
);

    logic sample_tick;
    
    baud_rate_generator #(
        .BAUD_DIV_WIDTH(16),
        .BAUD_DIV(325)
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
    } state_uart_r;
    statue_uart_r state, next_state;
    
    logic [DBITS -1:0] shift_reg, shift_reg_next;
    logic [3:0] s_cnt, s_cnt_next;
    logic [3:0] n_cnt, n_cnt_next;
    logic rx_done_next;
    
    localparam STOP_TICKS = SBITS * 16;
    
    always_ff @(posedge clock) begin : state_reg
        if(reset) begin
            state <= IDLE;
            shift_reg <= '0;
            s_cnt <= '0;
            n_cnt <= '0;
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
            
            STOP: begin
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

    logic baud_rate_tick;;
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
    statue_uart_t state, next_state;
    
    logic [7:0] tx_data;
    logic [2:0] data_cnt;
    logic start_trans, end_data, end_start, end_stop;
    
    always_ff @(posedge clock) begin : tx_data_reg
        if (reset)
            tx_data <= '0;
        else if (wr_uart && (state == IDLE))
            tx_data <= wr_data;
    end
    
    always_comb begin : tx_line
        case (state)
            START: tx <= 1'b0;
            DATA: tx <= tx_data[data_cnt];
            IDLE, STOP:  tx <= 1'b1;      
        endcase
    end

    always_ff @(posedge clock)begin : data_counter
        if (reset || end_data)
            data_cnt <= '0;
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
        next_state = start_trans ? START : state;
        next_state = end_start ? DATA : next_state;
        next_state = end_data ? STOP : next_state;
        next_state = end_stop ? IDLE : next_state;
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

endmodule

module rgb_controller (
    input logic clock,
    input logic reset,
    input logic [5:0] SW,
    output logic [2:0] RGB
);

    logic [1:0] red, green, blue;
    logic [1:0] pwm_count;
    
    assign red = SW[1:0];
    assign blue = SW[3:2];
    assign green = SW[5:4];
    
    always_ff @(posedge clock) begin
        if(reset) begin
            pwm_count <= 2'b0;
        end else begin
            pwm_count <= pwm_count + 1;
        end
    end
    
    assign RGB[2] = (pwm_count < red);
    assign RGB[1] = (pwm_count < green);
    assign RGB[0] = (pwm_count < blue);
    

endmodule

module top (
    input  logic        clk,        // System clock
    input  logic        rst,      // Active-low reset
    input  logic        uart_rx,    // UART receive line
    output logic        uart_tx,     // UART transmit line
    output logic        PWM_red,
    output logic        PWM_green,
    output logic        PWM_blue,
    output logic        done
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
    )uart_reciever (
        .clock     (clk),
        .reset     (rst),
        .rx        (uart_rx),
        .data_out  (rx_data),
        .rx_done (rx_done)       // Assuming rx_valid indicates if data is available
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


    // Control logic for RGB
    // on ctrl_valid signal we update the ctrl register 
    // ctrl register holds the 6 bit control signal for RGB controller 
    // extracted from the token
    logic [5:0] ctrl;
    logic ctrl_valid, ctrl_valid_next;

       // Instantiate the RGB controller

    logic rgb_controller_reset;
    logic rgb_controller_start;
    assign rgb_controller_reset = rst | rgb_controller_start;
    
    rgb_controller rgb_controller_inst (
        .clock  (clk),
        .reset  (rgb_controller_reset),
        .SW     (ctrl),
        .RGB    ({PWM_red, PWM_green, PWM_blue})
    );
 


    // Message bytes to be transmitted
    // "Ready\n" and "Control\n"
    logic [7:0] message_byte [0:15];
    initial begin
        message_byte[0]  = "R";
        message_byte[1]  = "e";
        message_byte[2]  = "a";
        message_byte[3]  = "d";
        message_byte[4]  = "y";
        message_byte[5]  = "\n";
        message_byte[6] = 8'h0; // Null terminator
        message_byte[7]  = "C";
        message_byte[8]  = "o";
        message_byte[9]  = "n";
        message_byte[10]  = "t";
        message_byte[11] = "r";
        message_byte[12] = "o";
        message_byte[13] = "l";
        message_byte[14] = "\n";
        message_byte[15] = 8'h0; // Null terminator
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
    logic [3:0] tx_index, tx_index_next;


    // State transition logic
    



    logic stop_flag;
    // stop flag is equal to 1 if we receive new line
    assign stop_flag = (rx_data == 8'h0A && rx_done) ? 1 : 0; 

    // Next state logic
    always_comb begin
        
    end





endmodule