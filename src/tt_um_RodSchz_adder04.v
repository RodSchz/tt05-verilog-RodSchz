module tt_um_RodSchz_adder04 (
    input  wire [7:0] ui_in,      // Dedicated inputs - Input for two values
    output wire [7:0] uo_out,     // Dedicated outputs - Sum results
    input  wire [7:0] uio_in,     // IOs: Bidirectional Input path
    output wire [7:0] uio_out,    // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,     // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    input  wire       ena,        // will go high when the design is enabled
    input  wire       clk,        // clock
    input  wire       rst_n       // reset_n - low to reset 
);
    wire [3:0] A = ui_in[7:4];
    wire [3:0] B = ui_in[3:0];
    reg  [3:0] SUM;

    assign uo_out[3:0] = SUM;

    assign uo_out[7:4] = 'b1011;
    assign uio_out[7:0] = 'b00000000;
    assign uio_oe[7:0] =  'b11111111;

    always @(A or B)
        SUM =  A + B;
    
endmodule
