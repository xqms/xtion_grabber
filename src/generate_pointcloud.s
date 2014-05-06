/* AVX accelerated pointcloud generation from depth & color images
 * Author: Max Schwarz <max.schwarz@online.de>
 *
 * FIXME: This currently produces pointclouds which are not directly usable
 *  by PCL. Our layout in 32 bit values: x y z [rgba].
 *          PCL layout in 32 bit values: x y z 0 [rgba] 0 0 0
 */

.intel_syntax noprefix


.section .text

.align 16
depth_mm_to_m: .float 0.001

.align 16
debug: .float 500.0

.globl __generatePointCloud_AVX
__generatePointCloud_AVX:
	#  signature: extern void __generatePointCloud_AVX(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output, const float* xlut, const float* ylut, uint64_t width, uint64_t height);
	# ARGUMENTS
	#  rdi depth
	#  rsi color
	#  rdx output
	#  rcx xlut
	#  r8  ylut
	#  r9  width
	#  [rsp] height

	# REGISTERS:
	#   xmm0 ZERO
	#   xmm1 DEPTH
	#   xmm2 X     or POINT0
	#   xmm3 Y     or POINT1
	#   xmm4 Z     or POINT2
	#   xmm5 DEPTH_PACKED (8 depth words)
	#   xmm6 DEPTH_INT    (4 depth 32-bit integers)
	#   xmm7 YLUT         (value of y lut for current y)
	#   xmm8 XLUT         (value of x lut for current x)
	#   xmm9 COLOR or POINT3
	#   xmm10 DEPTH_MM_TO_M (constant 0.001f)
	#   xmm11 TMP0
	#   xmm12 TMP1
	#   xmm13 TMP2
	#   xmm14 TMP3

	#   r10 running width
	#   r11 running height

	#   r9 color ptr
	#   r11 height (counted down)
	#   r12 width (counted down)

	mov r11, [rsp+8]

	# Save registers
	push rbx
	push r12

	# Setup constant registers
	vxorps xmm0, xmm0, xmm0 # ZERO

	vbroadcastss ymm10, [rip+depth_mm_to_m] # DEPTH_MM_TO_M

	# Loop over all y values
	loop_y:

		# Load YLUT value
		vbroadcastss xmm7, [r8]

		# Reset xlut pointer
		mov rbx, rcx

		# Loop over all x values
		mov r10, r9
		loop_x:

			# Load 8 depth words
			vmovaps xmm5, [rdi]

			# And unpack the lower half
			vpunpcklwd xmm6, xmm5, xmm0

			# Convert to float
			vcvtdq2ps xmm1, xmm6

			# Load color part
			vmovaps xmm9, [rsi]

			# Calculate point coordinates
			vmulps xmm2, xmm1, XMMWORD PTR [rbx]  # X = XLUT * DEPTH
			vmulps xmm3, xmm7, xmm1  # Y = YLUT * DEPTH
			vmulps xmm4, xmm10, xmm1 # Z = DEPTH_MM_TO_M * DEPTH

			# Transpose [2, 3, 4, 9] to get 4 separate points
			vunpcklps xmm11, xmm2, xmm3
			vunpcklps xmm13, xmm4, xmm9
			vunpckhps xmm12, xmm2, xmm3
			vunpckhps xmm14, xmm4, xmm9

			vmovlhps xmm2, xmm11, xmm13
			vmovhlps xmm3, xmm13, xmm11
			vmovlhps xmm4, xmm12, xmm14
			vmovhlps xmm9, xmm14, xmm12

			# Save 4 points
			vmovaps [rdx+0], xmm2
			vmovaps [rdx+16], xmm3
			vmovaps [rdx+32], xmm4
			vmovaps [rdx+48], xmm9

			# Increase pointers accordingly
			add rsi, 16
			add rdx, 64
			add rbx, 16

			# Now for the upper half
			vpunpckhwd xmm6, xmm5, xmm0

			# Convert to float
			vcvtdq2ps xmm1, xmm6

			# Load color part
			vmovaps xmm9, [rsi]

			# Calculate point coordinates
			vmulps xmm2, xmm1, XMMWORD PTR [rbx]  # X = XLUT * DEPTH
			vmulps xmm3, xmm7, xmm1  # Y = YLUT * DEPTH
			vmulps xmm4, xmm10, xmm1 # Z = DEPTH_MM_TO_M * DEPTH

			# Transpose [2, 3, 4, 9] to get 4 separate points
			vunpcklps xmm11, xmm2, xmm3
			vunpcklps xmm13, xmm4, xmm9
			vunpckhps xmm12, xmm2, xmm3
			vunpckhps xmm14, xmm4, xmm9

			vmovlhps xmm2, xmm11, xmm13
			vmovhlps xmm3, xmm13, xmm11
			vmovlhps xmm4, xmm12, xmm14
			vmovhlps xmm9, xmm14, xmm12

			# Save 4 points
			vmovaps [rdx+0], xmm2
			vmovaps [rdx+16], xmm3
			vmovaps [rdx+32], xmm4
			vmovaps [rdx+48], xmm9

			# Increase pointers accordingly
			add rdi, 16
			add rsi, 16
			add rdx, 64
			add rbx, 16

			# Do we need another x loop?
			add r10, -8
			test r10, r10
			jnz loop_x

		# Increase y pointer
		add r8, 4

		# Do we need another y loop?
		dec r11
		test r11, r11
		jnz loop_y

	pop r12
	pop rbx
	ret


.globl __generatePointCloud_AVX2
__generatePointCloud_AVX2:
	#  signature: extern void __generatePointCloud_AVX2(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output, const float* xlut, const float* ylut, uint64_t width, uint64_t height);
	# ARGUMENTS
	#  rdi depth
	#  rsi color
	#  rdx output
	#  rcx xlut
	#  r8  ylut
	#  r9  width
	#  [rsp] height

	# REGISTERS:
	#   ymm0 ZERO
	#   ymm1 DEPTH
	#   ymm2 X     or POINT0
	#   ymm3 Y     or POINT1
	#   ymm4 Z     or POINT2
	#   ymm5 DEPTH_PACKED (8 depth words)
	#   ymm6 DEPTH_INT    (4 depth 32-bit integers)
	#   ymm7 YLUT         (value of y lut for current y)
	#   ymm8 XLUT         (value of x lut for current x)
	#   ymm9 COLOR or POINT3
	#   ymm10 DEPTH_MM_TO_M (constant 0.001f)
	#   ymm11 TMP0
	#   ymm12 TMP1
	#   ymm13 TMP2
	#   ymm14 TMP3

	#   r10 running width
	#   r11 running height

	#   r9 color ptr
	#   r11 height (counted down)
	#   r12 width (counted down)

	mov r11, [rsp+8]

	# Save registers
	push rbx
	push r12

	# Setup constant registers
	vxorps ymm0, ymm0, ymm0 # ZERO

	vbroadcastss ymm10, [rip+depth_mm_to_m] # DEPTH_MM_TO_M
	vbroadcastss ymm1, [rip+debug]

	# Loop over all y values
	1:

		# Load YLUT value
		vbroadcastss ymm7, [r8]

		# Reset xlut pointer
		mov rbx, rcx

		# Loop over all x values
		mov r10, r9
		2:

			# Load 8 detph words and unpack them
			vpmovsxwd ymm6, [rdi]

			# Load color part
			vmovups ymm9, [rsi]

			/*# Load 8 depth words
			vmovaps xmm5, [rdi]

			# And unpack the lower half
			vpunpcklwd xmm2, xmm5, xmm0
			vpunpckhwd xmm3, xmm5, xmm0
			vinsertf128 ymm6, ymm2, xmm3, 1*/

			# Convert to float
			vcvtdq2ps ymm1, ymm6

			# Calculate point coordinates
			vmulps ymm2, ymm1, YMMWORD PTR [rbx]  # X = XLUT * DEPTH
			vmulps ymm3, ymm7, ymm1  # Y = YLUT * DEPTH
			vmulps ymm4, ymm10, ymm1 # Z = DEPTH_MM_TO_M * DEPTH

			# Big transpose 4 vectors of point elements -> 8 points
			vunpcklps ymm11, ymm2, ymm3
			vunpcklps ymm12, ymm4, ymm9
			vunpckhps ymm13, ymm2, ymm3
			vunpckhps ymm14, ymm4, ymm9

			vunpcklpd ymm2, ymm11, ymm12
			vunpcklpd ymm3, ymm13, ymm14
			vunpckhpd ymm4, ymm11, ymm12
			vunpckhpd ymm9, ymm13, ymm14

			# Store 8 points
			vmovups [rdx+0], ymm2
			vmovups [rdx+32], ymm3
			vmovups [rdx+64], ymm4
			vmovups [rdx+96], ymm9

			# Increase pointers accordingly
			add rdi, 16
			add rsi, 32
			add rdx, 128
			add rbx, 32

			# Do we need another x loop?
			add r10, -8
			test r10, r10
			jnz 2b

		# Increase y pointer
		add r8, 4

		# Do we need another y loop?
		dec r11
		test r11, r11
		jnz 1b

	pop r12
	pop rbx
	ret

.att_syntax noprefix
