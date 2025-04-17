module Execute_tb;
    // Định nghĩa các tín hiệu test
    reg [31:0] read_data1;      // Input từ Register File
    reg [31:0] read_data2;      // Input từ Register File
    reg [31:0] imm_gen_out;     // Input từ Immediate Generator
    reg ALUSrc;                 // Input từ Control Unit
    reg [1:0] ALUOp;           // Input từ Control Unit
    reg [2:0] funct3;          // Input từ Instruction
    reg [6:0] funct7;          // Input từ Instruction

    // Các tín hiệu output cần theo dõi
    wire [31:0] ALU_result;     // Output từ ALU
    wire zero_flag;             // Output Zero flag từ ALU
    wire [3:0] ALUcontrol_wire; // Output từ ALU Control
    wire [31:0] mux_out;        // Output từ ALU MUX

    // Khởi tạo các module cần test
    ALU_Control alu_ctrl(
        .funct3(funct3),
        .funct7(funct7),
        .ALUOp(ALUOp),
        .ALUcontrol_Out(ALUcontrol_wire)
    );

    MUX2to1 alu_mux(
        .input0(read_data2),
        .input1(imm_gen_out),
        .select(ALUSrc),
        .out(mux_out)
    );

    ALU alu(
        .A(read_data1),
        .B(mux_out),
        .ALUcontrol_In(ALUcontrol_wire),
        .Result(ALU_result),
        .Zero(zero_flag)
    );

    // Test cases
    initial begin
        // Khởi tạo file wave
        $dumpfile("execute_wave.vcd");
        $dumpvars(0, Execute_tb);

        // Test case 1: R-type ADD operation
        read_data1 = 32'h5;         // 5
        read_data2 = 32'h3;         // 3
        imm_gen_out = 32'h0;        // Not used
        ALUSrc = 0;                 // Select read_data2
        ALUOp = 2'b10;             // R-type
        funct3 = 3'b000;           // ADD/SUB
        funct7 = 7'b0000000;       // ADD
        #10;

        // Test case 2: R-type SUB operation
        funct7 = 7'b0100000;       // SUB
        #10;

        // Test case 3: R-type AND operation
        read_data1 = 32'hF;        // 1111
        read_data2 = 32'h3;        // 0011
        funct3 = 3'b111;           // AND
        funct7 = 7'b0000000;
        #10;

        // Test case 4: I-type ADDI operation
        ALUSrc = 1;                // Select immediate
        ALUOp = 2'b10;            // I-type
        funct3 = 3'b000;          // ADDI
        imm_gen_out = 32'h4;      // Immediate value = 4
        #10;

        // Test case 5: Branch Equal (BEQ)
        read_data1 = 32'h5;
        read_data2 = 32'h5;
        ALUSrc = 0;               // Select read_data2
        ALUOp = 2'b01;           // Branch
        funct3 = 3'b000;         // BEQ
        #10;

        // Test case 6: SLT operation
        read_data1 = 32'h3;
        read_data2 = 32'h5;
        ALUSrc = 0;
        ALUOp = 2'b10;
        funct3 = 3'b010;         // SLT
        funct7 = 7'b0000000;
        #10;

        // Kết thúc simulation
        #10;
        $finish;
    end

    // Monitor các tín hiệu quan trọng
    initial begin
        $monitor("Time=%0t\nALUOp=%b funct3=%b funct7=%b ALUcontrol=%b\nA=%h B=%h ALUSrc=%b imm=%h\nResult=%h Zero=%b\n",
                 $time, ALUOp, funct3, funct7, ALUcontrol_wire,
                 read_data1, read_data2, ALUSrc, imm_gen_out,
                 ALU_result, zero_flag);
    end

endmodule