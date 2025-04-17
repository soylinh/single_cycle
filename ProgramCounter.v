module program_counter(
    input clk,                
    input rst,                
    input [31:0] pc_in,       
    output reg [31:0] pc_out 
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_out <= 32'b00;  // Reset PC về 0
        end else begin
            pc_out <= pc_in;  // Cập nhật PC mới
        end
    end

endmodule