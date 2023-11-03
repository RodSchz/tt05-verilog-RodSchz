`default_nettype none
`timescale 1ns/1ps

/*
this testbench just instantiates the module and makes some convenient wires
that can be driven / tested by the cocotb test.py
*/

// testbench is controlled by test.py
module tb ();

    // this part dumps the trace to a vcd file that can be viewed with GTKWave
    initial begin
        $dumpfile ("tb.vcd");
        $dumpvars (0, tb);
        #1;
    end

    // wire up the inputs and outputs
    reg [3:0] a;
    reg [3:0] b;
    wire [7:0] sum;
    integer i;

    tt_um_RodSchz_adder04 example_adder (
    // include power ports for the Gate Level test
    `ifdef GL_TEST
        .VPWR( 1'b1),
        .VGND( 1'b0),
    `endif
        .ui_in      ({a,b}),    // Dedicated inputs
        .uo_out     (sum),      // Dedicated outputs
        );
initial begin
    a <= 0;
    b <= 0;

    for (i=0;i<5;i=i+1) begin
        #10 a <= $random;
            b <= $random;
    end
end
    
endmodule
