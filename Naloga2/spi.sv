module spiModule (
    input logic clk,
    input logic rst,
    input logic [7:0] data_in,
    input logic spi_miso,
    input logic start,
    input [15:0] limit,
    output logic spi_clk,
    output logic spi_mosi,
    output logic done,
    output logic [7:0] data_out
);




    // state machine states
    typedef enum logic [1:0] {
        IDLE,
        READ,
        WRITE
    } state_t;
    state_t current_state, next_state;


    logic done_next, spi_clk_next, spi_mosi_next;
    logic [15:0] clk_tick_counter, clk_tick_counter_next;
    logic [3:0] symbol_counter, symbol_counter_next;
    logic [7:0] shift_regI, shift_regI_next;
    logic [7:0] shift_regO, shift_regO_next;
    logic [7:0] data_out_next;
    
    //updateanje registrov
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
            spi_clk <= 1'b0;
            spi_mosi <= 1'b0;
            done <= 1'b0;
            clk_tick_counter <= 16'd0;
            symbol_counter <= 4'd0;
            shift_regI <= 8'd0;
            shift_regO <= 8'd0;
            data_out <= 8'd0;
            
        end else begin
            current_state <= next_state;
            spi_clk <= spi_clk_next;
            spi_mosi <= spi_mosi_next;
            done <= done_next;
            clk_tick_counter <= clk_tick_counter_next;
            symbol_counter <= symbol_counter_next;
            shift_regI <= shift_regI_next;
            shift_regO <= shift_regO_next;
            data_out <= data_out_next;
            
        end
    end
    
    // FSM
    always_comb begin
    
    next_state = current_state;
    done_next = 1'b0;
    spi_clk_next = spi_clk;
    spi_mosi_next = spi_mosi;
    clk_tick_counter_next = clk_tick_counter;
    symbol_counter_next = symbol_counter;
    shift_regI_next = shift_regI;
    shift_regO_next = shift_regO;
    
    case (current_state)
        IDLE: begin
            // SPI ura in stevci na 0
            spi_clk_next = 1'b0;
            clk_tick_counter_next = 16'd0;
            symbol_counter_next = 4'd0;
            
            //če je start naloži podatke v shitf reg in podaj prvi bit na MOSI
            if (start) begin
          
                shift_regO_next = data_in;
                spi_mosi_next = data_in[7];

                next_state = READ;
            end
        end
        
        READ: begin
            // ko ura pride do polovice periode SPI
            if (clk_tick_counter == limit -1) begin
                
                clk_tick_counter_next = 16'd0;
                spi_clk_next = ~spi_clk;// rising edge
                shift_regI_next = {shift_regI[6:0], spi_miso}; // sifti register na levo in sampli MISO
                symbol_counter_next = symbol_counter + 1;
                // vsi biti so sprejeti
                if (symbol_counter_next == 4'd8) begin
                    data_out_next = shift_regI_next;
                    done_next = 1'b1;
                    next_state = IDLE;
                
                end else begin
                    //še imamo podatke
                    next_state = WRITE;
                end
            end else begin
            //štejemo uro naprej
                clk_tick_counter_next = clk_tick_counter + 1;
            end
        end
        
        WRITE: begin
            //Pride do polovice periode SPI
            if (clk_tick_counter == limit - 1) begin
                
                clk_tick_counter_next = 16'd0;
                spi_clk_next = ~spi_clk;// faling edge
                shift_regO_next = {shift_regO[6:0], 1'b0}; // shifta output register in posodobi MOSI
                spi_mosi_next = shift_regO_next[7];
                //sampli now MISO
                next_state = READ;
            end else begin
            
                clk_tick_counter_next = clk_tick_counter + 1;
                
            end
        end
        
        default: begin
            next_state = IDLE;
            
        end
    endcase
    end
            
                
    
endmodule