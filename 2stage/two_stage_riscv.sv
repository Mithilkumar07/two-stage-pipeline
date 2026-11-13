module two_stage_riscv (
    input  logic        clk,
    input  logic        reset,

    // Instruction fetched from instruction memory at 'address' (stage 1)
    input  logic [31:0] instruction,

    // Data from data memory (load result)
    input  logic [31:0] mem_out,

    // CSR bank interface
    input  logic [31:0] mtvec,
    input  logic [31:0] mepc,
    input  logic [31:0] csr_rdata,
    input  logic [31:0] mie_out,
    input  logic        interrupt_pending,

    output logic        csr_en,
    output logic        trap_en,
    output logic        is_mret,
    output logic [31:0] trap_cause,
    output logic [31:0] trap_val,
    output logic [11:0] csr_addr,

    // Data memory interface (stage 2)
    output logic [31:0] rs2_data,
    output logic [31:0] alu_out,
    output logic        mem_write,
    output logic        mem_read,
    output logic [2:0]  fn3,
    output logic [6:0]  opcode,

    // PCs
    output logic [31:0] address,      // Stage 1 fetch PC  -> instruction memory
    output logic [31:0] pc_ex         // PC of instruction in stage 2 (for mepc)
);

    // =========================================================================
    // Internal interconnect
    // =========================================================================
    logic [31:0] rs1_data;
    logic [31:0] imm_out;
    logic [31:0] return_addr;
    logic [31:0] wb_data /* verilator public_flat_rd */;
    logic [31:0] pc_signed_offset;

    logic        and_out_ex;
    logic        branch;
    logic        alu_src;
    logic [1:0]  memtoreg;
    logic [2:0]  aluop;
    logic        fn7_5;
    logic        fn7_0;
    logic        mux_inp;

    logic [6:0]  fn7;
    logic [11:0] fn12;
    logic [6:0]  imm11_5;

    logic        sw_trap;
    logic        mem_write_raw;
    logic        mem_read_raw;

    logic        illegal_instr;
    logic        load_misaligned;
    logic        store_misaligned;
    logic        instr_misaligned;

    logic [31:0] actual_branch_target;
    logic [31:0] int_cause;

    // ---- Pipeline register outputs (stage 2 view) ----
    logic [31:0] instr_e;     // instruction being executed
    logic [31:0] pc_e;        // its PC
    logic        valid_e;     // 0 = bubble

    // An interrupt is only taken on a real instruction (not a bubble), so that
    // mepc always points at an instruction that will be re-executed after MRET.
    logic        irq_take;
    assign irq_take = interrupt_pending && valid_e;

    assign pc_ex = pc_e;

    // =========================================================================
    // Field extraction from the instruction in stage 2
    // =========================================================================
    assign fn7  = instr_e[31:25];
    assign fn12 = instr_e[31:20];

    // Memory enable gating
    assign mem_write = mem_write_raw && !trap_en;
    assign mem_read  = mem_read_raw  && !trap_en;

    // Branch / JALR target
    assign actual_branch_target = mux_inp ? {alu_out[31:1], 1'b0} : pc_signed_offset;

    // Instruction-address misalignment
    assign instr_misaligned =
        (and_out_ex || opcode == 7'h6F || opcode == 7'h67)
        && (actual_branch_target[1:0] != 2'b00);

    // Illegal instruction detection
    assign illegal_instr = !(
        (opcode == 7'h37) ||
        (opcode == 7'h17) ||
        (opcode == 7'h6F) ||
        (opcode == 7'h67 && fn3 == 3'h0) ||
        (opcode == 7'h63 && fn3 inside {3'h0, 3'h1, 3'h4, 3'h5, 3'h6, 3'h7}) ||
        (opcode == 7'h03 && fn3 inside {3'h0, 3'h1, 3'h2, 3'h4, 3'h5}) ||
        (opcode == 7'h23 && fn3 inside {3'h0, 3'h1, 3'h2}) ||
        (opcode == 7'h13) ||
        (opcode == 7'h33 && (fn7 == 7'h00 || fn7 == 7'h20 || fn7 == 7'h01)) ||
        (opcode == 7'h0F) ||
        (opcode == 7'h73 && fn3 != 3'h0) ||
        (opcode == 7'h73 && fn3 == 3'h0 &&
             fn12 inside {12'h000, 12'h001, 12'h302, 12'h105})
    );

    // Data alignment checks
    always_comb begin
        load_misaligned  = 1'b0;
        store_misaligned = 1'b0;

        if (opcode == 7'h03) begin
            case (fn3)
                3'h2:       if (alu_out[1:0] != 2'b00) load_misaligned = 1'b1;
                3'h1, 3'h5: if (alu_out[0])            load_misaligned = 1'b1;
                default:    load_misaligned = 1'b0;
            endcase
        end

        if (opcode == 7'h23) begin
            case (fn3)
                3'h2:    if (alu_out[1:0] != 2'b00) store_misaligned = 1'b1;
                3'h1:    if (alu_out[0])            store_misaligned = 1'b1;
                default: store_misaligned = 1'b0;
            endcase
        end
    end

    // Interrupt cause encoding
    always_comb begin
        int_cause = 32'h0;
        if (irq_take) begin
            if      (mie_out[11]) int_cause = {1'b1, 31'd11}; // MEI
            else if (mie_out[3])  int_cause = {1'b1, 31'd3};  // MSI
            else if (mie_out[7])  int_cause = {1'b1, 31'd7};  // MTI
        end
    end

    // Consolidated priority trap handler
    always_comb begin
        trap_en    = 1'b0;
        trap_cause = 32'h0;
        trap_val   = 32'h0;

        if (illegal_instr) begin
            trap_en    = 1'b1;
            trap_cause = 32'd2;
            trap_val   = instr_e;

        end else if (instr_misaligned) begin
            trap_en    = 1'b1;
            trap_cause = 32'd0;
            trap_val   = actual_branch_target;

        end else if (sw_trap) begin
            trap_en    = 1'b1;
            if (fn12 == 12'h000) begin
                trap_cause = 32'd11;
                trap_val   = 32'd0;
            end else begin
                trap_cause = 32'd3;
                trap_val   = pc_e;       // PC of the EBREAK in stage 2
            end

        end else if (load_misaligned) begin
            trap_en    = 1'b1;
            trap_cause = 32'd4;
            trap_val   = alu_out;

        end else if (store_misaligned) begin
            trap_en    = 1'b1;
            trap_cause = 32'd6;
            trap_val   = alu_out;

        end else if (irq_take) begin
            trap_en    = 1'b1;
            trap_cause = int_cause;
            trap_val   = 32'd0;
        end
    end

    // =========================================================================
    // STAGE 1: Instruction Fetch  (+ IF/EX pipeline register)
    // =========================================================================
    if_stage if_st (
        .clk              (clk),
        .reset            (reset),
        .stall            (1'b0),
        .instr_f          (instruction),
        .and_out          (and_out_ex),
        .pc_signed_offset (pc_signed_offset),
        .mepc             (mepc),
        .mtvec            (mtvec),
        .trap_cause       (trap_cause),
        .trap_occurred    (trap_en),
        .is_mret_instr    (is_mret),
        .address          (address),
        .instr_e          (instr_e),
        .pc_e             (pc_e),
        .pc_plus4_e       (return_addr),
        .valid_e          (valid_e)
    );

    // =========================================================================
    // STAGE 2: Decode + Execute + Memory + Write-back
    // =========================================================================
    id_stage id_st (
        .clk              (clk),
        .reset            (reset),
        .instruction      (instr_e),
        .wb_data          (wb_data),
        .rs1_data         (rs1_data),
        .rs2_data         (rs2_data),
        .imm_out          (imm_out),
        .opcode_out_d     (opcode),
        .imm11_5          (imm11_5),
        .fn3_out_d        (fn3),
        .fn7_5            (fn7_5),
        .fn7_0            (fn7_0),
        .aluop            (aluop),
        .memtoreg         (memtoreg),
        .branch           (branch),
        .mem_write        (mem_write_raw),
        .mem_read         (mem_read_raw),
        .alu_src          (alu_src),
        .mux_inp          (mux_inp),
        .csr_we           (csr_en),
        .is_mret          (is_mret),
        .sw_trap          (sw_trap),
        .csr_addr         (csr_addr),
        .trap_en          (trap_en)
    );

    ex_stage ex_st (
        .rs1_data         (rs1_data),
        .rs2_data         (rs2_data),
        .imm_out          (imm_out),
        .address          (pc_e),
        .csr_read_data    (csr_rdata),
        .branch           (branch),
        .alu_src          (alu_src),
        .fn7_5            (fn7_5),
        .fn7_0            (fn7_0),
        .mux_inp          (mux_inp),
        .fn3              (fn3),
        .imm11_5          (imm11_5),
        .aluop            (aluop),
        .opcode           (opcode),
        .and_out_ex       (and_out_ex),
        .alu_out          (alu_out),
        .pc_ex_out        (pc_signed_offset)
    );

    wb_stage wb_st (
        .mem_out          (mem_out),
        .alu_out          (alu_out),
        .csr_read_data    (csr_rdata),
        .return_addr      (return_addr),
        .imm_out          (imm_out),
        .pc_signed_offset (pc_signed_offset),
        .memtoreg         (memtoreg),
        .opcode_out_d     (opcode),
        .wb_data          (wb_data)
    );

endmodule