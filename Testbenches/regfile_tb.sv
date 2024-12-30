module regfile_tb ();

    reg [15:0] data_in;
    reg [2:0] writenum, readnum;
    reg write;
    reg clk = 0;
    reg err = 0;
    wire [15:0] data_out;

    regfile DUT (.data_in(data_in), .writenum(writenum), .write(write), .readnum(readnum), .clk(clk), .data_out(data_out));

    task my_checker;

        input [15:0] write_in;
        input [2:0] writen, readn;
        input [15:0] expected_reading;

        data_in = write_in;
        writenum = writen;
        readnum = readn;

        #3;
        clk = 1;
        #3;
        clk = 0;

        if (data_out !== expected_reading) begin
            $display("ERROR ** reading is %b, expected %b",
            data_out, expected_reading);
            err = 1'b1;
        end else begin
            $display("SUCCESS! %b is the correct reading", expected_reading);
        end
    endtask

    initial begin
        write = 1;
        my_checker(16'b00000000000000001, 3'b000, 3'b000, 16'b00000000000000001); //Changing then reading R0
        my_checker(16'b00000000000000110, 3'b001, 3'b000, 16'b00000000000000001); //Changing R1 then reading R0 to check if it stays 0
        write = 0;
        my_checker(16'b1111111111111111, 3'b010, 3'b001, 16'b00000000000000110); //Trying not to write into R2 and then reading R1 
        write = 1;
        my_checker(16'b1111111111111111, 3'b000, 3'b000, 16'b1111111111111111); //Making sure that R2 was not written into and writing and reading into R0
        my_checker(16'b1111111111111110, 3'b011, 3'b011, 16'b1111111111111110); //Write and read into R3
        my_checker(16'b1111111111111100, 3'b100, 3'b100, 16'b1111111111111100); //Write and read into R4
        my_checker(16'b1111111111111000, 3'b101, 3'b101, 16'b1111111111111000); //Write and read into R5
        my_checker(16'b1111111111110000, 3'b110, 3'b110, 16'b1111111111110000); //Write and read into R6
        my_checker(16'b1111111111100000, 3'b111, 3'b111, 16'b1111111111100000); //Write and read into R7
        my_checker(16'b1111111111111111, 3'b010, 3'b111, 16'b1111111111100000); //Writing into R2 then checking that R7 remained the same
    end
endmodule
