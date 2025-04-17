module Memory_tb;
    // Định nghĩa các tín hiệu test
    reg clk;
    reg rst;
    reg MemRead;           // Input từ Control Unit
    reg MemWrite;          // Input từ Control Unit
    reg [31:0] address;    // Input từ ALU Result
    reg [31:0] write_data; // Input từ Register File (rs2)

    // Các tín hiệu output cần theo dõi
    wire [31:0] read_data; // Output từ Data Memory

    // Khởi tạo module Data Memory
    Data_Memory data_mem(
        .clk(clk),
        .rst(rst),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .address(address),
        .write_data(write_data),
        .read_data(read_data)
    );

    // Tạo clock
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // Test cases
    initial begin
        // Khởi tạo file wave
        $dumpfile("memory_wave.vcd");
        $dumpvars(0, Memory_tb);

        // Test case 1: Reset
        rst = 1;
        MemRead = 0;
        MemWrite = 0;
        address = 32'h0;
        write_data = 32'h0;
        #20;

        // Test case 2: Write to memory (sw instruction)
        rst = 0;
        MemWrite = 1;
        MemRead = 0;
        address = 32'h8;         // Địa chỉ memory = 8
        write_data = 32'h1234;   // Data = 0x1234
        #20;

        // Test case 3: Read from memory (lw instruction)
        MemWrite = 0;
        MemRead = 1;
        address = 32'h8;         // Đọc từ địa chỉ vừa ghi
        #20;

        // Test case 4: Read from predefined memory location
        address = 32'h17;        // Đọc từ địa chỉ 17 (có giá trị 56)
        #20;

        // Test case 5: Write then Read
        MemWrite = 1;
        MemRead = 0;
        address = 32'h10;
        write_data = 32'h5678;
        #20;

        MemWrite = 0;
        MemRead = 1;
        #20;

        // Test case 6: Multiple writes
        MemWrite = 1;
        MemRead = 0;
        address = 32'h20;
        write_data = 32'hABCD;
        #20;

        address = 32'h24;
        write_data = 32'hDEF0;
        #20;

        // Test case 7: Read without MemRead enabled
        MemWrite = 0;
        MemRead = 0;
        address = 32'h20;
        #20;

        // Test case 8: Read from unwritten address
        MemRead = 1;
        address = 32'h30;    // Địa chỉ chưa được ghi
        #20;

        // Test case 9: Write to predefined location
        MemWrite = 1;
        MemRead = 0;
        address = 32'h15;    // Địa chỉ predefined (65)
        write_data = 32'h9999;
        #20;

        MemWrite = 0;
        MemRead = 1;
        #20;

        // Kết thúc simulation
        #20;
        $finish;
    end

    // Monitor các tín hiệu quan trọng
    initial begin
        $monitor("Time=%0t\nMemRead=%b MemWrite=%b\nAddress=%h Write_Data=%h\nRead_Data=%h\n",
                 $time,
                 MemRead, MemWrite,
                 address, write_data,
                 read_data);
    end

    // Kiểm tra lỗi
    always @(posedge clk) begin
        // Kiểm tra đọc/ghi cùng lúc
        if (MemRead && MemWrite)
            $display("Warning: MemRead and MemWrite are both active!");
        
        // Kiểm tra địa chỉ hợp lệ
        if (address >= 64)
            $display("Warning: Address out of range!");
    end

endmodule