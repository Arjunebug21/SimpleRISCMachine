module shifter_tb();
    reg [15:0] in;
    reg [1:0] shift;
    wire [15:0] sout;
    reg err = 0;

    shifter DUT (.in(in), .shift(shift), .sout(sout));

    task shifter_checker;

    input [15:0] in_code;
    input [1:0] shift_code;
    input [15:0] expected_output;

    in = in_code;
    shift = shift_code;
    #5;

    if (sout !== expected_output) begin
        $display("**ERROR** output is %b and should be %b", sout, expected_output);
        err = 1;
    end else begin
        $display("SUCCESS! output change to %b works", sout);
    end
    endtask

    initial begin
        shifter_checker(16'b1001001001001001, 2'b00, 16'b1001001001001001); //Check that output = input
        shifter_checker(16'b1111000011001111, 2'b01, 16'b1110000110011110); //These three use table 3 from the lab manual
        shifter_checker(16'b1111000011001111, 2'b10, 16'b0111100001100111);
        shifter_checker(16'b1111000011001111, 2'b11, 16'b1111100001100111);
        shifter_checker({16{1'b1}}, 2'b01, {{15{1'b1}}, 1'b0}); //Check that left shift works
        shifter_checker({16{1'b1}}, 2'b10, {1'b0, {15{1'b1}}}); //Check that right shift works
        shifter_checker({16{1'b1}}, 2'b11, {16{1'b1}}); //Check that right shift works then copy 1 to MSB
        shifter_checker({1'b0, {15{1'b1}}}, 2'b11, {{2{1'b0}}, {14{1'b1}}}); //Check that right shift works then copy 0 to MSB
    end

endmodule
