module MUX2to1_DataMemory (
    input [31:0] input0,  
    input [31:0] input1,  
    input select,         
    output [31:0] out     
);

    assign out = (select) ? input1 : input0; 

endmodule