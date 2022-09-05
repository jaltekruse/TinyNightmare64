/*
 * Do not edit this file.  It was automatically generated
 * by "rgb2c" from the file "Textures/field32.rgb".
 *
/*
 *   Size: 32 x 32
 *   Number of channels: 4
 *   Number of bits per texel: 16 (G_IM_SIZ_16b)
 *   Format of texel: G_IM_FMT_RGBA
 *
 * Example usage:
 *
 *   gsSPTexture (128, 128, (levels-1), G_TX_RENDERTILE, 1),
 *   gsDPPipeSync (),
 *   gsDPSetCombineMode (G_CC_MODULATERGBA, G_CC_MODULATERGBA),
 *   gsDPSetTexturePersp (G_TP_PERSP),
 *   gsDPSetTextureDetail (G_TD_CLAMP),
 *   gsDPSetTextureLOD (G_TL_TILE),
 *   gsDPSetTextureLUT (G_TT_NONE),
 *   gsDPSetTextureFilter (G_TF_BILERP),
 *   gsDPSetTextureConvert(G_TC_FILT),
 *   gsDPLoadTextureBlock (RGBA16field32, G_IM_FMT_RGBA, G_IM_SIZ_16b, 32, 32, 0
 *     G_TX_WRAP | G_TX_NOMIRROR, G_TX_WRAP | G_TX_NOMIRROR,
 *     G_TX_NOMASK, G_TX_NOMASK, G_TX_NOLOD, G_TX_NOLOD),
 */

#if 0	/* Image preview */
	+--------------------------------+
	|+oo+++++++++++++++++++++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooo+o~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|++ooooooooo~~~~~~~~~~~++++++++++|
	|~+ooooooooo~~~~~~~~~~~++++++~~~+|
	|~+ooooooooo~~~~~~~~~~~++++++++~+|
	|~+ooooooooo~~~~~~~~~~~+++++++~~+|
	|~+oooooooo+~~~~~~~~~~~++++++++~~|
	|~+ooo++o+++~~~~~~~~~~~++++++~+~+|
	|++oooooooooo+~~~~~~~~+++++++++++|
	|+~~~~~~~~~++~~~~~~~~~+++++++++++|
	|+~~~~~~~~~~~~~~~~~~~~~++++~~+++~|
	|+~~~~~~~~~~~~~~~~~~~~~+~~~~~~~+~|
	|+~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~+~~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~+~~~~~+~~~|
	|+~~~~~~~~~~~~~~~~~~~~~++~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~++~~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~+++~~~~~~~|
	|+~~~~~~~~~~~~~~~~~~~~~++~~~+~+~~|
	|+~~~~~~~~~~~~~~~~~~~~~+++++++~~~|
	|+~~~~~~~~~~~~~~~~~~~~~+++++++~~+|
	|+~~~~~~~~~~+~~~~~~~~~~++++++++++|
	+--------------------------------+
#endif

static Gfx RGBA16field32_MIP_dummy_aligner1[] = { gsSPEndDisplayList() };

unsigned short RGBA16field32_orig[] = {
0x7b93, 0x5b4f, 0x4acf, 0x530f, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x5b11, 0x73d3, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acd, 0x52cf, 0x4acf, 0x52cf, 0x73d1, 0x7351, 0x7391, 0x6b51, 0x6b91, 0x7351, 0x6b51, 0x6b4f, 0x7351, 0x7351, 
0x6b91, 0x4ad1, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x4b0f, 0x5311, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x52cf, 0x4acf, 0x4acf, 0x52cf, 0x6b91, 0x6b91, 0x6b51, 0x6351, 0x634f, 0x6b51, 0x6351, 0x630f, 0x634f, 0x6311, 
0x6b91, 0x4ad1, 0x4b0f, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x530f, 0x4ad1, 0x4acf, 0x4acf, 0x4acf, 0x4acd, 0x4acf, 0x4acf, 0x52cd, 0x4acf, 0x5b0f, 0x6b93, 0x6b51, 0x6351, 0x6b51, 0x6351, 0x634f, 0x6351, 0x634f, 0x6311, 0x634f, 
0x6b91, 0x4ad1, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x42cd, 0x4acf, 0x428f, 0x4acf, 0x530f, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x4acd, 0x4acf, 0x4acf, 0x52cd, 0x4acf, 0x4acf, 0x5b0f, 0x6b93, 0x6351, 0x634f, 0x6351, 0x634f, 0x6351, 0x5b0f, 0x6b51, 0x634f, 0x6351, 
0x6b51, 0x4acf, 0x4b0f, 0x4acf, 0x4ad1, 0x42cf, 0x4b0f, 0x428d, 0x4acf, 0x4acf, 0x4acf, 0x530f, 0x4acf, 0x4acf, 0x4acf, 0x4a8d, 0x4acf, 0x4acd, 0x42cf, 0x4acd, 0x52cf, 0x530f, 0x6b53, 0x6391, 0x6351, 0x6311, 0x5b4f, 0x6351, 0x5b0f, 0x634f, 0x630f, 0x6351, 
0x6b51, 0x4acf, 0x4b0f, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x4acf, 0x530f, 0x42cf, 0x4acf, 0x4acd, 0x4acf, 0x4acd, 0x52cf, 0x4acf, 0x4acd, 0x4acf, 0x634f, 0x6351, 0x6351, 0x5b51, 0x6351, 0x5b0f, 0x5b51, 0x6351, 0x5b0f, 0x630f, 0x5b51, 
0x6351, 0x4acf, 0x5311, 0x4acf, 0x430f, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4a8f, 0x4b0f, 0x4acf, 0x52cf, 0x4acf, 0x4acf, 0x4acd, 0x4acf, 0x4acf, 0x4a8f, 0x4acf, 0x4acd, 0x52cf, 0x634f, 0x5b11, 0x5b11, 0x5351, 0x5b11, 0x5b0f, 0x5b0f, 0x5311, 0x630f, 0x5311, 
0x6b51, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4b0f, 0x530f, 0x4acf, 0x4acf, 0x52cf, 0x4b0f, 0x4a8f, 0x4acd, 0x4acf, 0x4acd, 0x4a8d, 0x530f, 0x6351, 0x6351, 0x5b11, 0x634f, 0x5311, 0x6351, 0x530f, 0x5b51, 0x630f, 0x530f, 
0x6b51, 0x42cf, 0x4acf, 0x4b0f, 0x4ad1, 0x4acf, 0x42cf, 0x4acd, 0x4ad1, 0x4acf, 0x4b0f, 0x52cf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acd, 0x4acf, 0x4acd, 0x428d, 0x4a8d, 0x6b51, 0x5b4f, 0x5b11, 0x5b51, 0x5b4f, 0x6351, 0x6351, 0x5b0f, 0x5b0f, 0x5b0f, 
0x6351, 0x4acf, 0x4acf, 0x4acf, 0x430f, 0x4a8f, 0x4acf, 0x42cd, 0x4acf, 0x4acf, 0x4acf, 0x530f, 0x42cf, 0x4acf, 0x4a8d, 0x4acf, 0x4acf, 0x4acf, 0x4a8d, 0x4acd, 0x4acf, 0x52cf, 0x634f, 0x5b11, 0x5b11, 0x5b4f, 0x5b51, 0x5b11, 0x5b0f, 0x6351, 0x5b4f, 0x5b11, 
0x6351, 0x4acf, 0x4acf, 0x4b0f, 0x42cf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x4acf, 0x4b0f, 0x4a8f, 0x4acd, 0x4acf, 0x4acf, 0x4a8d, 0x4acf, 0x4acd, 0x4a8d, 0x4b0f, 0x52cf, 0x634f, 0x5b11, 0x5b51, 0x5b11, 0x5b51, 0x5b0f, 0x5311, 0x5b11, 0x5b4f, 0x5b0f, 
0x6351, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x4acf, 0x4b0f, 0x52cf, 0x42cf, 0x4acf, 0x4a8d, 0x4acf, 0x4acd, 0x4acf, 0x428d, 0x4acd, 0x4acf, 0x52cf, 0x6351, 0x5b11, 0x5b4f, 0x5b11, 0x5b51, 0x5b51, 0x5b4f, 0x6351, 0x5b51, 0x5b0f, 
0x6351, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x42cf, 0x4acf, 0x5311, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4a8f, 0x42cd, 0x4acf, 0x4a8d, 0x530f, 0x6351, 0x5b0f, 0x5b11, 0x534f, 0x5b11, 0x6351, 0x5b51, 0x5b51, 0x6351, 0x630f, 
0x6b51, 0x42cf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4acf, 0x42cf, 0x4acf, 0x4a8d, 0x5311, 0x4b0f, 0x4acf, 0x4acf, 0x4acf, 0x4acf, 0x4a8d, 0x428d, 0x4acf, 0x4acd, 0x4acf, 0x52cf, 0x6b91, 0x6351, 0x6351, 0x6351, 0x6351, 0x5b51, 0x6351, 0x6351, 0x6351, 0x634f, 
0x7391, 0x5311, 0x5b0f, 0x5b11, 0x5b11, 0x530f, 0x5b11, 0x5b0f, 0x5b11, 0x5b0f, 0x6b51, 0x6351, 0x5b11, 0x5b11, 0x5b0f, 0x630f, 0x5b11, 0x6311, 0x630f, 0x5b4f, 0x6311, 0x7351, 0x7bd1, 0x6b93, 0x7391, 0x6b91, 0x6b91, 0x6b91, 0x6b53, 0x7391, 0x6b91, 0x6b51, 
0x7391, 0x7351, 0x9c57, 0x9c55, 0x9c57, 0x9c15, 0x9457, 0x9c55, 0x9415, 0x9457, 0x9c15, 0x9c99, 0x6b51, 0x5b4f, 0x6351, 0x5b0f, 0x6351, 0x630f, 0x5b11, 0x5b4f, 0x630f, 0x6b51, 0x7bd3, 0x6b93, 0x7391, 0x6b93, 0x6b91, 0x6b51, 0x6b51, 0x6351, 0x6b91, 0x6b51, 
0x6351, 0x734f, 0x9c55, 0x9417, 0x9415, 0x9415, 0x8c15, 0x9415, 0x9413, 0x8c15, 0x8c15, 0x630f, 0x42cf, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428b, 0x4acf, 0x7391, 0x5b51, 0x6351, 0x6351, 0x6351, 0x6b51, 0x6351, 0x634f, 0x6311, 0x634f, 
0x630f, 0x7391, 0x9c15, 0x9457, 0x9c55, 0x9415, 0x9c17, 0x9455, 0x9c15, 0x9415, 0x9415, 0x4acf, 0x4acd, 0x428d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x4a8d, 0x428b, 0x52cf, 0x6b91, 0x6353, 0x6391, 0x6b51, 0x6351, 0x6351, 0x6351, 0x6b51, 0x5b11, 0x634f, 
0x634f, 0x7351, 0x9c55, 0x9c57, 0x9c57, 0x9415, 0x9c57, 0x9c57, 0x9c57, 0x9c15, 0x9457, 0x4a8d, 0x42cd, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x4a8d, 0x428b, 0x4acf, 0x6b91, 0x6351, 0x6b93, 0x6351, 0x6351, 0x6b51, 0x6351, 0x634f, 0x6311, 0x634f, 
0x630f, 0x7b93, 0x9c55, 0x9c55, 0x9c57, 0x9c57, 0x9457, 0x9c57, 0x9c57, 0x9c57, 0x9c55, 0x4acf, 0x4a8d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x4acf, 0x6b91, 0x6b91, 0x6b51, 0x6351, 0x6351, 0x6351, 0x6b51, 0x6351, 0x630f, 0x6351, 
0x6351, 0x7b91, 0x9c55, 0xa457, 0x9c57, 0x9c57, 0x9c57, 0x9c57, 0x9c57, 0x9c57, 0x9c55, 0x4a8f, 0x42cf, 0x4a8d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428b, 0x428d, 0x4a8d, 0x4acd, 0x6b93, 0x6351, 0x6b91, 0x6351, 0x6b51, 0x6351, 0x6351, 0x630f, 0x634f, 0x6311, 
0x6351, 0x7b8f, 0xa457, 0x9c57, 0xa497, 0xa457, 0xa457, 0xa457, 0x9c57, 0x9c57, 0x9c55, 0x4acf, 0x4a8f, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x52cf, 0x6b91, 0x7391, 0x6b51, 0x6b51, 0x6b51, 0x6b51, 0x6b51, 0x6b4f, 0x6b51, 0x6b4f, 
0x7351, 0x7b91, 0xa497, 0x9c57, 0xa497, 0xa497, 0x9c57, 0xa457, 0xa497, 0x9c17, 0x9c57, 0x4acf, 0x4acd, 0x4acf, 0x428d, 0x42cf, 0x4a8d, 0x428d, 0x428d, 0x428d, 0x428d, 0x52cf, 0x73d3, 0x6b51, 0x7391, 0x6b51, 0x7351, 0x6b51, 0x6b51, 0x6b91, 0x6b0f, 0x6b4f, 
0x6b4f, 0x8393, 0x9c57, 0xa497, 0xa497, 0xa457, 0x9c97, 0xa457, 0xa457, 0x9c57, 0x9c55, 0x4acf, 0x4acf, 0x4acd, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x4acd, 0x73d3, 0x7b93, 0x6b51, 0x6b51, 0x7391, 0x6b51, 0x6b91, 0x7351, 0x6b51, 0x7351, 
0x7351, 0x7b91, 0xa497, 0xa499, 0xa497, 0xa459, 0xa497, 0xa497, 0xa457, 0x9c57, 0x9c57, 0x4acf, 0x4acd, 0x4acf, 0x428d, 0x428d, 0x428d, 0x42cf, 0x428d, 0x428d, 0x428d, 0x52cf, 0x73d1, 0x6b53, 0x7391, 0x6b91, 0x7351, 0x7353, 0x6b91, 0x6b51, 0x6b51, 0x734f, 
0x6b51, 0x7b91, 0xa497, 0xa499, 0xa457, 0xa497, 0xa459, 0xa497, 0x9c55, 0x9c57, 0x9c55, 0x4acf, 0x428d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x428d, 0x428d, 0x52cf, 0x73d1, 0x6b51, 0x7351, 0x7393, 0x7391, 0x7351, 0x6b91, 0x6351, 0x734f, 0x7351, 
0x7b91, 0x7b91, 0xa457, 0x9c97, 0x9c55, 0xa457, 0xa457, 0x9c55, 0x9c55, 0x9c57, 0x9455, 0x4acf, 0x4a8f, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x428d, 0x428d, 0x52cf, 0x8413, 0x83d3, 0x83d3, 0x83d3, 0x7b91, 0x7b93, 0x7b91, 0x7391, 0x7b91, 0x7b91, 
0x7b91, 0x7b91, 0xa457, 0x9c55, 0xa497, 0x9c57, 0x9c55, 0x9c55, 0x9c55, 0x9c17, 0x9455, 0x4acd, 0x428f, 0x428d, 0x428d, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x52cf, 0x7bd3, 0x7bd3, 0x8391, 0x7b91, 0x7b91, 0x7b93, 0x7b91, 0x7b91, 0x7b91, 0x7b91, 
0x7b91, 0x7b91, 0xa457, 0x9c97, 0x9c55, 0x9c57, 0x9c55, 0x9c55, 0x9c55, 0x9415, 0x9415, 0x4acd, 0x4a8f, 0x428d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x530f, 0x8391, 0x7bd3, 0x7b91, 0x7bd3, 0x7b91, 0x7b93, 0x7b91, 0x7b91, 0x7b91, 0x7b91, 
0x7b91, 0x7b91, 0x9c57, 0x9c57, 0x9c55, 0x9c57, 0x9c55, 0x9c55, 0x9417, 0x9c55, 0x9415, 0x52cf, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x4a8d, 0x428d, 0x4a8d, 0x428d, 0x428d, 0x52cf, 0x83d3, 0x7bd3, 0x7b91, 0x7bd3, 0x7b91, 0x7b51, 0x7b91, 0x7391, 0x7b91, 0x7351, 
0x8391, 0x83d3, 0x9c57, 0x9c57, 0x9c57, 0x9c57, 0x9c57, 0xacd9, 0x9c17, 0x9c55, 0x9415, 0x5b0f, 0x428d, 0x4a8f, 0x42cd, 0x4a8d, 0x428d, 0x4a8f, 0x428d, 0x4acd, 0x4a8d, 0x5b0f, 0x83d3, 0x7bd3, 0x8391, 0x83d3, 0x7b91, 0x7b93, 0x7b91, 0x7b91, 0x7b91, 0x7b91, 
0x9415, 0x9c55, 0x9457, 0x8c57, 0x83d3, 0x73d3, 0x6b93, 0x7393, 0x6b93, 0x6b91, 0x7393, 0x8c15, 0x6b51, 0x7351, 0x6b51, 0x7391, 0x6b91, 0x7391, 0x6351, 0x7351, 0x6351, 0x7b91, 0x83d5, 0x83d3, 0x83d3, 0x83d3, 0x83d3, 0x83d3, 0x83d3, 0x7b93, 0x8c13, 0x83d3, 
};

static Gfx RGBA16field32_dummy_aligner2[] = { gsSPEndDisplayList() };

static unsigned short RGBA16field32_buf[2744] = {
	0x7b93, 0x5b4f, 0x4acf, 0x530f, 	0x4acf, 0x42cf, 0x4acf, 0x4acf, 
	0x42cf, 0x4acf, 0x5b11, 0x73d3, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x4acf, 0x4acd, 0x52cf, 	0x4acf, 0x52cf, 0x73d1, 0x7351, 
	0x7391, 0x6b51, 0x6b91, 0x7351, 	0x6b51, 0x6b4f, 0x7351, 0x7351, 
	0x4acf, 0x4b0f, 0x6b91, 0x4ad1, 	0x4acf, 0x42cf, 0x4acf, 0x4acf, 
	0x4b0f, 0x5311, 0x4acf, 0x4acf, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x52cf, 0x4acf, 0x4acf, 0x4acf, 	0x6b91, 0x6b91, 0x4acf, 0x52cf, 
	0x634f, 0x6b51, 0x6b51, 0x6351, 	0x634f, 0x6311, 0x6351, 0x630f, 
	0x6b91, 0x4ad1, 0x4b0f, 0x4acf, 	0x4b0f, 0x4acf, 0x4acf, 0x4acf, 
	0x42cf, 0x4acf, 0x4acf, 0x530f, 	0x4ad1, 0x4acf, 0x4acf, 0x4acf, 
	0x4acd, 0x4acf, 0x4acf, 0x52cd, 	0x4acf, 0x5b0f, 0x6b93, 0x6b51, 
	0x6351, 0x6b51, 0x6351, 0x634f, 	0x6351, 0x634f, 0x6311, 0x634f, 
	0x4acf, 0x4b0f, 0x6b91, 0x4ad1, 	0x42cd, 0x4acf, 0x4acf, 0x4acf, 
	0x530f, 0x4b0f, 0x428f, 0x4acf, 	0x4acf, 0x4acd, 0x4acf, 0x4acf, 
	0x52cd, 0x4acf, 0x4acf, 0x4acf, 	0x6b93, 0x6351, 0x4acf, 0x5b0f, 
	0x634f, 0x6351, 0x634f, 0x6351, 	0x634f, 0x6351, 0x5b0f, 0x6b51, 
	0x6b51, 0x4acf, 0x4b0f, 0x4acf, 	0x4ad1, 0x42cf, 0x4b0f, 0x428d, 
	0x4acf, 0x4acf, 0x4acf, 0x530f, 	0x4acf, 0x4acf, 0x4acf, 0x4a8d, 
	0x4acf, 0x4acd, 0x42cf, 0x4acd, 	0x52cf, 0x530f, 0x6b53, 0x6391, 
	0x6351, 0x6311, 0x5b4f, 0x6351, 	0x5b0f, 0x634f, 0x630f, 0x6351, 
	0x4b0f, 0x4acf, 0x6b51, 0x4acf, 	0x4acf, 0x42cf, 0x4b0f, 0x4acf, 
	0x4acf, 0x530f, 0x4acf, 0x4acf, 	0x4acd, 0x4acf, 0x42cf, 0x4acf, 
	0x4acf, 0x4acd, 0x4acd, 0x52cf, 	0x6351, 0x6351, 0x4acf, 0x634f, 
	0x5b0f, 0x5b51, 0x5b51, 0x6351, 	0x630f, 0x5b51, 0x6351, 0x5b0f, 
	0x6351, 0x4acf, 0x5311, 0x4acf, 	0x430f, 0x4acf, 0x4acf, 0x42cf, 
	0x4acf, 0x4a8f, 0x4b0f, 0x4acf, 	0x52cf, 0x4acf, 0x4acf, 0x4acd, 
	0x4acf, 0x4acf, 0x4a8f, 0x4acf, 	0x4acd, 0x52cf, 0x634f, 0x5b11, 
	0x5b11, 0x5351, 0x5b11, 0x5b0f, 	0x5b0f, 0x5311, 0x630f, 0x5311, 
	0x4b0f, 0x4acf, 0x6b51, 0x4acf, 	0x4acf, 0x4acf, 0x4acf, 0x4b0f, 
	0x4b0f, 0x530f, 0x4acf, 0x4acf, 	0x52cf, 0x4b0f, 0x4acf, 0x4acf, 
	0x4acf, 0x4acd, 0x4a8f, 0x4acd, 	0x6351, 0x6351, 0x4a8d, 0x530f, 
	0x5311, 0x6351, 0x5b11, 0x634f, 	0x630f, 0x530f, 0x530f, 0x5b51, 
	0x6b51, 0x42cf, 0x4acf, 0x4b0f, 	0x4ad1, 0x4acf, 0x42cf, 0x4acd, 
	0x4ad1, 0x4acf, 0x4b0f, 0x52cf, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x4acd, 0x4acf, 0x4acd, 	0x428d, 0x4a8d, 0x6b51, 0x5b4f, 
	0x5b11, 0x5b51, 0x5b4f, 0x6351, 	0x6351, 0x5b0f, 0x5b0f, 0x5b0f, 
	0x4acf, 0x4acf, 0x6351, 0x4acf, 	0x4acf, 0x42cd, 0x430f, 0x4a8f, 
	0x4acf, 0x530f, 0x4acf, 0x4acf, 	0x4a8d, 0x4acf, 0x42cf, 0x4acf, 
	0x4a8d, 0x4acd, 0x4acf, 0x4acf, 	0x634f, 0x5b11, 0x4acf, 0x52cf, 
	0x5b51, 0x5b11, 0x5b11, 0x5b4f, 	0x5b4f, 0x5b11, 0x5b0f, 0x6351, 
	0x6351, 0x4acf, 0x4acf, 0x4b0f, 	0x42cf, 0x4acf, 0x4acf, 0x42cf, 
	0x4acf, 0x4acf, 0x4acf, 0x4b0f, 	0x4a8f, 0x4acd, 0x4acf, 0x4acf, 
	0x4a8d, 0x4acf, 0x4acd, 0x4a8d, 	0x4b0f, 0x52cf, 0x634f, 0x5b11, 
	0x5b51, 0x5b11, 0x5b51, 0x5b0f, 	0x5311, 0x5b11, 0x5b4f, 0x5b0f, 
	0x4acf, 0x4acf, 0x6351, 0x4acf, 	0x42cf, 0x4acf, 0x4acf, 0x4acf, 
	0x4b0f, 0x52cf, 0x4acf, 0x4acf, 	0x4a8d, 0x4acf, 0x42cf, 0x4acf, 
	0x428d, 0x4acd, 0x4acd, 0x4acf, 	0x6351, 0x5b11, 0x4acf, 0x52cf, 
	0x5b51, 0x5b51, 0x5b4f, 0x5b11, 	0x5b51, 0x5b0f, 0x5b4f, 0x6351, 
	0x6351, 0x4acf, 0x4acf, 0x4acf, 	0x4acf, 0x4acf, 0x42cf, 0x4acf, 
	0x42cf, 0x4acf, 0x5311, 0x4b0f, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x4a8f, 0x42cd, 0x4acf, 	0x4a8d, 0x530f, 0x6351, 0x5b0f, 
	0x5b11, 0x534f, 0x5b11, 0x6351, 	0x5b51, 0x5b51, 0x6351, 0x630f, 
	0x4acf, 0x4acf, 0x6b51, 0x42cf, 	0x4acf, 0x42cf, 0x42cf, 0x4acf, 
	0x5311, 0x4b0f, 0x4acf, 0x4a8d, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x4acd, 0x4a8d, 0x428d, 	0x6b91, 0x6351, 0x4acf, 0x52cf, 
	0x6351, 0x5b51, 0x6351, 0x6351, 	0x6351, 0x634f, 0x6351, 0x6351, 
	0x7391, 0x5311, 0x5b0f, 0x5b11, 	0x5b11, 0x530f, 0x5b11, 0x5b0f, 
	0x5b11, 0x5b0f, 0x6b51, 0x6351, 	0x5b11, 0x5b11, 0x5b0f, 0x630f, 
	0x5b11, 0x6311, 0x630f, 0x5b4f, 	0x6311, 0x7351, 0x7bd1, 0x6b93, 
	0x7391, 0x6b91, 0x6b91, 0x6b91, 	0x6b53, 0x7391, 0x6b91, 0x6b51, 
	0x9c57, 0x9c55, 0x7391, 0x7351, 	0x9457, 0x9c55, 0x9c57, 0x9c15, 
	0x9c15, 0x9c99, 0x9415, 0x9457, 	0x6351, 0x5b0f, 0x6b51, 0x5b4f, 
	0x5b11, 0x5b4f, 0x6351, 0x630f, 	0x7bd3, 0x6b93, 0x630f, 0x6b51, 
	0x6b91, 0x6b51, 0x7391, 0x6b93, 	0x6b91, 0x6b51, 0x6b51, 0x6351, 
	0x6351, 0x734f, 0x9c55, 0x9417, 	0x9415, 0x9415, 0x8c15, 0x9415, 
	0x9413, 0x8c15, 0x8c15, 0x630f, 	0x42cf, 0x428d, 0x428d, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x428d, 	0x428b, 0x4acf, 0x7391, 0x5b51, 
	0x6351, 0x6351, 0x6351, 0x6b51, 	0x6351, 0x634f, 0x6311, 0x634f, 
	0x9c15, 0x9457, 0x630f, 0x7391, 	0x9c17, 0x9455, 0x9c55, 0x9415, 
	0x9415, 0x4acf, 0x9c15, 0x9415, 	0x428d, 0x428d, 0x4acd, 0x428d, 
	0x428d, 0x4a8d, 0x428d, 0x4a8d, 	0x6b91, 0x6353, 0x428b, 0x52cf, 
	0x6351, 0x6351, 0x6391, 0x6b51, 	0x5b11, 0x634f, 0x6351, 0x6b51, 
	0x634f, 0x7351, 0x9c55, 0x9c57, 	0x9c57, 0x9415, 0x9c57, 0x9c57, 
	0x9c57, 0x9c15, 0x9457, 0x4a8d, 	0x42cd, 0x428d, 0x428d, 0x428d, 
	0x4a8d, 0x428d, 0x428d, 0x4a8d, 	0x428b, 0x4acf, 0x6b91, 0x6351, 
	0x6b93, 0x6351, 0x6351, 0x6b51, 	0x6351, 0x634f, 0x6311, 0x634f, 
	0x9c55, 0x9c55, 0x630f, 0x7b93, 	0x9457, 0x9c57, 0x9c57, 0x9c57, 
	0x9c55, 0x4acf, 0x9c57, 0x9c57, 	0x428d, 0x428d, 0x4a8d, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x428d, 	0x6b91, 0x6b91, 0x4a8d, 0x4acf, 
	0x6351, 0x6351, 0x6b51, 0x6351, 	0x630f, 0x6351, 0x6b51, 0x6351, 
	0x6351, 0x7b91, 0x9c55, 0xa457, 	0x9c57, 0x9c57, 0x9c57, 0x9c57, 
	0x9c57, 0x9c57, 0x9c55, 0x4a8f, 	0x42cf, 0x4a8d, 0x428d, 0x428d, 
	0x4a8d, 0x428d, 0x428b, 0x428d, 	0x4a8d, 0x4acd, 0x6b93, 0x6351, 
	0x6b91, 0x6351, 0x6b51, 0x6351, 	0x6351, 0x630f, 0x634f, 0x6311, 
	0xa457, 0x9c57, 0x6351, 0x7b8f, 	0xa457, 0xa457, 0xa497, 0xa457, 
	0x9c55, 0x4acf, 0x9c57, 0x9c57, 	0x428d, 0x428d, 0x4a8f, 0x428d, 
	0x428d, 0x428d, 0x428d, 0x428d, 	0x6b91, 0x7391, 0x428d, 0x52cf, 
	0x6b51, 0x6b51, 0x6b51, 0x6b51, 	0x6b51, 0x6b4f, 0x6b51, 0x6b4f, 
	0x7351, 0x7b91, 0xa497, 0x9c57, 	0xa497, 0xa497, 0x9c57, 0xa457, 
	0xa497, 0x9c17, 0x9c57, 0x4acf, 	0x4acd, 0x4acf, 0x428d, 0x42cf, 
	0x4a8d, 0x428d, 0x428d, 0x428d, 	0x428d, 0x52cf, 0x73d3, 0x6b51, 
	0x7391, 0x6b51, 0x7351, 0x6b51, 	0x6b51, 0x6b91, 0x6b0f, 0x6b4f, 
	0x9c57, 0xa497, 0x6b4f, 0x8393, 	0x9c97, 0xa457, 0xa497, 0xa457, 
	0x9c55, 0x4acf, 0xa457, 0x9c57, 	0x428d, 0x428d, 0x4acf, 0x4acd, 
	0x428d, 0x4a8d, 0x428d, 0x428d, 	0x73d3, 0x7b93, 0x428d, 0x4acd, 
	0x7391, 0x6b51, 0x6b51, 0x6b51, 	0x6b51, 0x7351, 0x6b91, 0x7351, 
	0x7351, 0x7b91, 0xa497, 0xa499, 	0xa497, 0xa459, 0xa497, 0xa497, 
	0xa457, 0x9c57, 0x9c57, 0x4acf, 	0x4acd, 0x4acf, 0x428d, 0x428d, 
	0x428d, 0x42cf, 0x428d, 0x428d, 	0x428d, 0x52cf, 0x73d1, 0x6b53, 
	0x7391, 0x6b91, 0x7351, 0x7353, 	0x6b91, 0x6b51, 0x6b51, 0x734f, 
	0xa497, 0xa499, 0x6b51, 0x7b91, 	0xa459, 0xa497, 0xa457, 0xa497, 
	0x9c55, 0x4acf, 0x9c55, 0x9c57, 	0x428d, 0x428d, 0x428d, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x428d, 	0x73d1, 0x6b51, 0x428d, 0x52cf, 
	0x7391, 0x7351, 0x7351, 0x7393, 	0x734f, 0x7351, 0x6b91, 0x6351, 
	0x7b91, 0x7b91, 0xa457, 0x9c97, 	0x9c55, 0xa457, 0xa457, 0x9c55, 
	0x9c55, 0x9c57, 0x9455, 0x4acf, 	0x4a8f, 0x428d, 0x428d, 0x428d, 
	0x4a8d, 0x428d, 0x428d, 0x428d, 	0x428d, 0x52cf, 0x8413, 0x83d3, 
	0x83d3, 0x83d3, 0x7b91, 0x7b93, 	0x7b91, 0x7391, 0x7b91, 0x7b91, 
	0xa457, 0x9c55, 0x7b91, 0x7b91, 	0x9c55, 0x9c55, 0xa497, 0x9c57, 
	0x9455, 0x4acd, 0x9c55, 0x9c17, 	0x428d, 0x428d, 0x428f, 0x428d, 
	0x4a8d, 0x428d, 0x428d, 0x428d, 	0x7bd3, 0x7bd3, 0x428d, 0x52cf, 
	0x7b91, 0x7b93, 0x8391, 0x7b91, 	0x7b91, 0x7b91, 0x7b91, 0x7b91, 
	0x7b91, 0x7b91, 0xa457, 0x9c97, 	0x9c55, 0x9c57, 0x9c55, 0x9c55, 
	0x9c55, 0x9415, 0x9415, 0x4acd, 	0x4a8f, 0x428d, 0x428d, 0x428d, 
	0x4a8d, 0x428d, 0x428d, 0x4a8d, 	0x428d, 0x530f, 0x8391, 0x7bd3, 
	0x7b91, 0x7bd3, 0x7b91, 0x7b93, 	0x7b91, 0x7b91, 0x7b91, 0x7b91, 
	0x9c57, 0x9c57, 0x7b91, 0x7b91, 	0x9c55, 0x9c55, 0x9c55, 0x9c57, 
	0x9415, 0x52cf, 0x9417, 0x9c55, 	0x428d, 0x428d, 0x428d, 0x4a8d, 
	0x4a8d, 0x428d, 0x4a8d, 0x428d, 	0x83d3, 0x7bd3, 0x428d, 0x52cf, 
	0x7b91, 0x7b51, 0x7b91, 0x7bd3, 	0x7b91, 0x7351, 0x7b91, 0x7391, 
	0x8391, 0x83d3, 0x9c57, 0x9c57, 	0x9c57, 0x9c57, 0x9c57, 0xacd9, 
	0x9c17, 0x9c55, 0x9415, 0x5b0f, 	0x428d, 0x4a8f, 0x42cd, 0x4a8d, 
	0x428d, 0x4a8f, 0x428d, 0x4acd, 	0x4a8d, 0x5b0f, 0x83d3, 0x7bd3, 
	0x8391, 0x83d3, 0x7b91, 0x7b93, 	0x7b91, 0x7b91, 0x7b91, 0x7b91, 
	0x9457, 0x8c57, 0x9415, 0x9c55, 	0x6b93, 0x7393, 0x83d3, 0x73d3, 
	0x7393, 0x8c15, 0x6b93, 0x6b91, 	0x6b51, 0x7391, 0x6b51, 0x7351, 
	0x6351, 0x7351, 0x6b91, 0x7391, 	0x83d5, 0x83d3, 0x6351, 0x7b91, 
	0x83d3, 0x83d3, 0x83d3, 0x83d3, 	0x8c13, 0x83d3, 0x83d3, 0x7b93, 
	0x7393, 0x6351, 0x5b11, 0x5311, 	0x530f, 0x6351, 0x5b11, 0x530f, 
	0x530f, 0x530f, 0x530f, 0x7391, 	0x7391, 0x7391, 0x7351, 0x7351, 
	0x4acf, 0x4acf, 0x6351, 0x4acf, 	0x4acf, 0x4acf, 0x4acf, 0x4b0f, 
	0x52cf, 0x6351, 0x4acf, 0x4acf, 	0x6351, 0x634f, 0x6351, 0x6351, 
	0x6351, 0x4acf, 0x4acf, 0x4acf, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x4acf, 0x52cf, 0x6351, 	0x6351, 0x6351, 0x634f, 0x634f, 
	0x4acf, 0x4acf, 0x5b11, 0x4b0f, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x6351, 0x4acf, 0x4acf, 	0x5b0f, 0x5b0f, 0x5b51, 0x5b11, 
	0x5b11, 0x4acf, 0x4acf, 0x4acf, 	0x4acf, 0x4b0f, 0x4acf, 0x4acf, 
	0x4acf, 0x4acd, 0x4acd, 0x5b0f, 	0x5b11, 0x5b51, 0x5b51, 0x5b0f, 
	0x4acf, 0x4acf, 0x5b11, 0x4acf, 	0x4acf, 0x4acf, 0x4acf, 0x4acf, 
	0x4acf, 0x5b0f, 0x4acf, 0x4acd, 	0x5b11, 0x5b4f, 0x5b11, 0x5b51, 
	0x5b11, 0x4acf, 0x4acf, 0x4acf, 	0x4acf, 0x4b0f, 0x4acf, 0x4acf, 
	0x4acf, 0x4acd, 0x4acf, 0x5b51, 	0x5b51, 0x5b51, 0x5b51, 0x6351, 
	0x6351, 0x6351, 0x6b51, 0x6351, 	0x5b51, 0x5b0f, 0x6351, 0x6b51, 
	0x5b0f, 0x7391, 0x5b0f, 0x5b0f, 	0x6b51, 0x6b91, 0x6b91, 0x6b91, 
	0x6b51, 0x9415, 0x9415, 0x9415, 	0x9415, 0x8bd5, 0x530f, 0x4acd, 
	0x4acd, 0x4acd, 0x4acd, 0x6b51, 	0x6351, 0x6351, 0x6351, 0x6351, 
	0x9c57, 0x9c57, 0x6b4f, 0x9415, 	0x4acd, 0x428d, 0x9c57, 0x83d3, 
	0x4a8d, 0x6351, 0x428d, 0x428d, 	0x6351, 0x6351, 0x6b51, 0x6351, 
	0x6b51, 0x9415, 0x9c57, 0x9c57, 	0x9c57, 0x8bd5, 0x4a8f, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x6351, 	0x6b51, 0x6b51, 0x6351, 0x634f, 
	0xa497, 0xa457, 0x7351, 0x9c55, 	0x4acf, 0x428d, 0xa457, 0x8bd5, 
	0x4a8d, 0x6b91, 0x428d, 0x428d, 	0x6b51, 0x6b51, 0x6b51, 0x6b51, 
	0x7351, 0x9c57, 0xa497, 0xa497, 	0xa457, 0x8c15, 0x4acf, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x6b91, 	0x7391, 0x7351, 0x6b91, 0x6b51, 
	0xa457, 0xa457, 0x7b91, 0x9c55, 	0x4a8f, 0x428d, 0x9c55, 0x83d3, 
	0x4a8d, 0x7391, 0x428d, 0x428d, 	0x7391, 0x7b91, 0x7b93, 0x7b91, 
	0x7b91, 0x9415, 0x9c57, 0x9c55, 	0x9c55, 0x83d3, 0x4a8d, 0x428d, 
	0x428d, 0x428d, 0x4a8d, 0x7391, 	0x7b91, 0x7b91, 0x7b91, 0x7b91, 
	0x9457, 0x9455, 0x83d3, 0x9457, 	0x52cf, 0x52cf, 0x9415, 0x83d3, 
	0x52cf, 0x7b93, 0x52cf, 0x52cf, 	0x7b91, 0x7b91, 0x83d3, 0x7b93, 
	0x7391, 0x6351, 0x6351, 0x5b11, 	0x530f, 0x5b0f, 0x7391, 0x7351, 
	0x4acf, 0x4acf, 0x5b11, 0x4acf, 	0x6351, 0x634f, 0x4acf, 0x530f, 
	0x5b11, 0x4acf, 0x4acf, 0x4acf, 	0x4acf, 0x52cf, 0x5b11, 0x5b51, 
	0x530f, 0x530f, 0x5b11, 0x530f, 	0x6351, 0x6351, 0x52cf, 0x530f, 
	0x7391, 0x8bd5, 0x8bd5, 0x5b0f, 	0x4acd, 0x530f, 0x6b51, 0x6351, 
	0x9c57, 0x5b0f, 0x7391, 0x9c57, 	0x6b51, 0x6351, 0x428d, 0x52cf, 
	0x7b93, 0xa497, 0x9c57, 0x5b0f, 	0x428d, 0x52cf, 0x7391, 0x7391, 
	0x9415, 0x5acf, 0x83d3, 0x9c57, 	0x7b91, 0x7b91, 0x4a8d, 0x52cf, 
	0x7391, 0x6351, 0x52cf, 0x6b51, 	0x4acf, 0x5b11, 0x5b11, 0x4acf, 
	0x7391, 0x7b93, 0x52cf, 0x6351, 	0x4acf, 0x6b51, 0x83d3, 0x8c15, 
	0x6b51, 0x5b11, 0x0000, 0x0000, 	0x0000, 0x0000, 0x6b91, 0x5b11, 
	0x6351, 0x0000, 0x0000, 0x0000, };

Gfx RGBA16field32_dl[] = {
	gsDPSetTextureImage( 0, 2, 1, RGBA16field32_buf),
	gsDPSetTile( 0, 2, 0, 0, G_TX_LOADTILE, 0, 0, 0, 0, 0, 0, 0),
	gsDPLoadBlock( G_TX_LOADTILE, 0, 0, 1372, 0),
	gsDPSetTile(0, 2, 8, 0, 0, 0, 0, 5, 0, 0, 5, 0),
	gsDPSetTileSize( 0,  0, 0, 31 << G_TEXTURE_IMAGE_FRAC, 31 << G_TEXTURE_IMAGE_FRAC),
	gsDPSetTile(0, 2, 4, 256, 1, 0, 0, 4, 1, 0, 4, 1),
	gsDPSetTileSize( 1,  0, 0, 15 << G_TEXTURE_IMAGE_FRAC, 15 << G_TEXTURE_IMAGE_FRAC),
	gsDPSetTile(0, 2, 2, 320, 2, 0, 0, 3, 2, 0, 3, 2),
	gsDPSetTileSize( 2,  0, 0, 7 << G_TEXTURE_IMAGE_FRAC, 7 << G_TEXTURE_IMAGE_FRAC),
	gsDPSetTile(0, 2, 1, 336, 3, 0, 0, 2, 3, 0, 2, 3),
	gsDPSetTileSize( 3,  0, 0, 3 << G_TEXTURE_IMAGE_FRAC, 3 << G_TEXTURE_IMAGE_FRAC),
	gsDPSetTile(0, 2, 1, 340, 4, 0, 0, 1, 4, 0, 1, 4),
	gsDPSetTileSize( 4,  0, 0, 1 << G_TEXTURE_IMAGE_FRAC, 1 << G_TEXTURE_IMAGE_FRAC),
	gsDPSetTile(0, 2, 1, 342, 5, 0, 0, 0, 5, 0, 0, 5),
	gsDPSetTileSize( 5,  0, 0, 0 << G_TEXTURE_IMAGE_FRAC, 0 << G_TEXTURE_IMAGE_FRAC),
	gsSPEndDisplayList(),
};
