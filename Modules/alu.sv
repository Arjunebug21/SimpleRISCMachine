module ALU (Ain, Bin, ALUop, out, N_sig, V_sig, Z_sig);
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output reg [15:0] out;
    output wire N_sig, V_sig, Z_sig;

    always_comb begin
        case (ALUop) 
        2'b00: out = Ain + Bin;
        2'b01: out = Ain - Bin;
        2'b10: out = Ain & Bin;
        2'b11: out = ~Bin;
        default : out = 16'bxxxxxxxxxxxxxxxx;
        endcase
    end

    assign Z_sig = (out === 16'b0000000000000000) ? 1'b1 : 1'b0;

    assign N_sig = out[15]; 

    assign V_sig = ((Ain[15] === Bin[15]) && (out[15] !== Ain[15])) ? 1'b1: 1'b0;

endmodule
