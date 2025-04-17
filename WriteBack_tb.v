module WriteBack_tb;
    // Định nghĩa các tín hiệu test
    reg [31:0] ALU_result;      // Input từ ALU
    reg [31:0] mem_data;        // Input từ Data Memory
    reg MemtoReg;              // Input từ Control Unit
    
    // Các tín hiệu output cần theo dõi
    wire [31:0] write_data;     // Output để ghi vào Register File

    // Khởi tạo module Write Back MUX
    MUX2to1_DataMemory WB_Mux(
        .input0(ALU_result),
        .input1(mem_data),
        .select(MemtoReg),
        .out(write_data)
    );

    // Test cases
    initial begin
        // Khởi tạo file wave
        $dumpfile("writeback_wave.vcd");
        $dumpvars(0, WriteBack_tb);

        // Test case 1: Chọn ALU Result (R-type, I-type instructions)
        ALU_result = 32'h12345678;
        mem_data = 32'h87654321;
        MemtoReg = 0;           // Chọn ALU Result
        #10;

        // Test case 2: Chọn Memory Data (lw instruction)
        MemtoReg = 1;           // Chọn Memory Data
        #10;

        // Test case 3: ALU Result với giá trị 0
        ALU_result = 32'h0;
        mem_data = 32'hAAAAAAAA;
        MemtoReg = 0;
        #10;

        // Test case 4: Memory Data với giá trị 0
        ALU_result = 32'hBBBBBBBB;
        mem_data = 32'h0;
        MemtoReg = 1;
        #10;

        // Test case 5: Giá trị âm từ ALU
        ALU_result = 32'hFFFFFFFF;  // -1 trong complementary 2
        mem_data = 32'h11111111;
        MemtoReg = 0;
        #10;

        // Test case 6: Giá trị âm từ Memory
        ALU_result = 32'h22222222;
        mem_data = 32'hFFFFFFFF;    // -1 trong complementary 2
        MemtoReg = 1;
        #10;

        // Test case 7: Chuyển đổi nhanh giữa ALU và Memory
        MemtoReg = 0;
        #5;
        MemtoReg = 1;
        #5;
        MemtoReg = 0;
        #5;
        MemtoReg = 1;
        #5;

        // Test case 8: Giá trị lớn
        ALU_result = 32'hFFFFFFFF;
        mem_data = 32'hFFFFFFFF;
        MemtoReg = 0;
        #10;
        MemtoReg = 1;
        #10;

        // Kết thúc simulation
        #10;
        $finish;
    end

    // Monitor các tín hiệu quan trọng
    initial begin
        $monitor("Time=%0t\nMemtoReg=%b\nALU_Result=%h\nMemory_Data=%h\nWrite_Data=%h\n",
                 $time,
                 MemtoReg,
                 ALU_result,
                 mem_data,
                 write_data);
    end

    // Kiểm tra tính hợp lệ của output
    always @(write_data) begin
        // Kiểm tra xem output có khớp với input được chọn không
        if (MemtoReg && (write_data !== mem_data))
            $display("Error: Write data mismatch when selecting Memory data!");
        else if (!MemtoReg && (write_data !== ALU_result))
            $display("Error: Write data mismatch when selecting ALU result!");
    end

endmodule