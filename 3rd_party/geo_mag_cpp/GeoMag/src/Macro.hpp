/**
 * @file Macro.hpp
 * @author Kaiji Takeuchi
 * @brief マクロ魔術
 * @version 0.1
 * @date 2023-04-30
 * @copyright Copyright (c) 2023
 */

#pragma once

// clang-format off
#define GEOMAG_VERSION_MAJOR 1
#define GEOMAG_VERSION_MINOR 0
#define GEOMAG_VERSION_PATCH 1

#define GEOMAG_TO_STRING_EX(x) #x
#define GEOMAG_TO_STRING(x) GEOMAG_TO_STRING_EX(x)

#define GEOMAG_VERSION_CONCAT_EX(major, minor, patch) v ## major.minor.patch
#define GEOMAG_VERSION_CONCAT(major, minor, patch) GEOMAG_VERSION_CONCAT_EX(major, minor, patch)

#define GEOMAG_VERSION_EX \
	GEOMAG_VERSION_CONCAT(GEOMAG_VERSION_MAJOR, \
				      GEOMAG_VERSION_MINOR, \
			              GEOMAG_VERSION_PATCH)
#define GEOMAG_VERSION GEOMAG_VERSION_EX
#define GEOMAG_VERSION_STR_EX GEOMAG_TO_STRING(GEOMAG_VERSION)
#define GEOMAG_VERSION_STR GEOMAG_VERSION_STR_EX

#define GEOMAG_NAMESPACE_VERSION_TAG_CONCAT_EX(major, minor, patch) v ## major ## _ ## minor ## _ ## patch
#define GEOMAG_NAMESPACE_VERSION_TAG_CONCAT(major, minor, patch) GEOMAG_NAMESPACE_VERSION_TAG_CONCAT_EX(major, minor, patch)
#define GEOMAG_NAMESPACE_VERSION_TAG_EX \
	GEOMAG_NAMESPACE_VERSION_TAG_CONCAT(GEOMAG_VERSION_MAJOR, \
						    GEOMAG_VERSION_MINOR, \
					            GEOMAG_VERSION_PATCH)
#define GEOMAG_NAMESPACE_BASE_TAG geomag
#define GEOMAG_NAMESPACE_VERSION_TAG GEOMAG_NAMESPACE_VERSION_TAG_EX
#define GEOMAG_NAMESPACE_BEGIN \
	namespace GEOMAG_NAMESPACE_BASE_TAG {                   \
		inline namespace GEOMAG_NAMESPACE_VERSION_TAG {
#define GEOMAG_NAMESPACE_END \
	}                                               \
}

#define GEOMAG_REQUEST_VERSION_CHECK(major, minor, patch) \
	(GEOMAG_VERSION_MAJOR >= major && GEOMAG_VERSION_MINOR >= minor && GEOMAG_VERSION_PATCH >= patch)

#define GEOMAG_REQUEST_VERSION_ASSERTION_MSG_STR(major, minor, patch) "GeMag-Lib must be has version higher than " GEOMAG_TO_STRING(GEOMAG_VERSION_CONCAT(major, minor, patch))

#ifndef GEOMAG_NO_ASSERTION
#define GEOMAG_ASSRET_CONVERTER_REQUEST_VERSION(major, minor, patch) \
	static_assert(GEOMAG_REQUEST_VERSION_CHECK(major, minor, patch), GEOMAG_REQUEST_VERSION_ASSERTION_MSG_STR(major, minor, patch))
#else
	TELEMETRY_ASSRET_CONVERTER_REQUEST_VERSION(major, minor, patch)
#endif

#define GEOMAG_CODE_GEN_CONCAT_EX(tag, type) tag ## _ ## type
#define GEOMAG_CODE_GEN_CONCAT(tag, type) GEOMAG_CODE_GEN_CONCAT_EX(tag, type)
#define GEOMAG_CODE_GEN_TAG koyoh_acs_GEOMAG_code_gen
#define GEOMAG_CODE_GEN_RESULT_FUNCTION_NAME(x) GEOMAG_CODE_GEN_CONCAT(GEOMAG_CODE_GEN_TAG, x)
#define GEOMAG_CODE_GEN_ARG_STR_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, str_t)
#define GEOMAG_CODE_GEN_ARG_OBJ_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, obj_t)
#define GEOMAG_CODE_GEN_ARG_PTR_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, ptr_t)
#define GEOMAG_CODE_GEN_ARG_OFS_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, ofs_t)
#define GEOMAG_CODE_GEN_ARG_IPT_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, ipt_t)
#define GEOMAG_CODE_GEN_ARG_OPT_T GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, opt_t)
#define GEOMAG_CODE_GEN_TEMPLATE_TYPE Type
#define GEOMAG_CODE_GEN_TARGET_OBJ_NAME GEOMAG_CODE_GEN_CONCAT(GEOMAG_NAMESPACE_BASE_TAG, obj_name)
#define GEOMAG_CODE_GEN_ARG_EXPAND( x ) x
#define GEOMAG_CODE_GEN_ARG_GET_MACRO(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62, _63, _64, NAME,...) NAME
#define GEOMAG_CODE_GEN_ARG_PASTE(...) GEOMAG_CODE_GEN_ARG_EXPAND(GEOMAG_CODE_GEN_ARG_GET_MACRO(__VA_ARGS__, \
        GEOMAG_CODE_GEN_ARG_PASTE64, \
        GEOMAG_CODE_GEN_ARG_PASTE63, \
        GEOMAG_CODE_GEN_ARG_PASTE62, \
        GEOMAG_CODE_GEN_ARG_PASTE61, \
        GEOMAG_CODE_GEN_ARG_PASTE60, \
        GEOMAG_CODE_GEN_ARG_PASTE59, \
        GEOMAG_CODE_GEN_ARG_PASTE58, \
        GEOMAG_CODE_GEN_ARG_PASTE57, \
        GEOMAG_CODE_GEN_ARG_PASTE56, \
        GEOMAG_CODE_GEN_ARG_PASTE55, \
        GEOMAG_CODE_GEN_ARG_PASTE54, \
        GEOMAG_CODE_GEN_ARG_PASTE53, \
        GEOMAG_CODE_GEN_ARG_PASTE52, \
        GEOMAG_CODE_GEN_ARG_PASTE51, \
        GEOMAG_CODE_GEN_ARG_PASTE50, \
        GEOMAG_CODE_GEN_ARG_PASTE49, \
        GEOMAG_CODE_GEN_ARG_PASTE48, \
        GEOMAG_CODE_GEN_ARG_PASTE47, \
        GEOMAG_CODE_GEN_ARG_PASTE46, \
        GEOMAG_CODE_GEN_ARG_PASTE45, \
        GEOMAG_CODE_GEN_ARG_PASTE44, \
        GEOMAG_CODE_GEN_ARG_PASTE43, \
        GEOMAG_CODE_GEN_ARG_PASTE42, \
        GEOMAG_CODE_GEN_ARG_PASTE41, \
        GEOMAG_CODE_GEN_ARG_PASTE40, \
        GEOMAG_CODE_GEN_ARG_PASTE39, \
        GEOMAG_CODE_GEN_ARG_PASTE38, \
        GEOMAG_CODE_GEN_ARG_PASTE37, \
        GEOMAG_CODE_GEN_ARG_PASTE36, \
        GEOMAG_CODE_GEN_ARG_PASTE35, \
        GEOMAG_CODE_GEN_ARG_PASTE34, \
        GEOMAG_CODE_GEN_ARG_PASTE33, \
        GEOMAG_CODE_GEN_ARG_PASTE32, \
        GEOMAG_CODE_GEN_ARG_PASTE31, \
        GEOMAG_CODE_GEN_ARG_PASTE30, \
        GEOMAG_CODE_GEN_ARG_PASTE29, \
        GEOMAG_CODE_GEN_ARG_PASTE28, \
        GEOMAG_CODE_GEN_ARG_PASTE27, \
        GEOMAG_CODE_GEN_ARG_PASTE26, \
        GEOMAG_CODE_GEN_ARG_PASTE25, \
        GEOMAG_CODE_GEN_ARG_PASTE24, \
        GEOMAG_CODE_GEN_ARG_PASTE23, \
        GEOMAG_CODE_GEN_ARG_PASTE22, \
        GEOMAG_CODE_GEN_ARG_PASTE21, \
        GEOMAG_CODE_GEN_ARG_PASTE20, \
        GEOMAG_CODE_GEN_ARG_PASTE19, \
        GEOMAG_CODE_GEN_ARG_PASTE18, \
        GEOMAG_CODE_GEN_ARG_PASTE17, \
        GEOMAG_CODE_GEN_ARG_PASTE16, \
        GEOMAG_CODE_GEN_ARG_PASTE15, \
        GEOMAG_CODE_GEN_ARG_PASTE14, \
        GEOMAG_CODE_GEN_ARG_PASTE13, \
        GEOMAG_CODE_GEN_ARG_PASTE12, \
        GEOMAG_CODE_GEN_ARG_PASTE11, \
        GEOMAG_CODE_GEN_ARG_PASTE10, \
        GEOMAG_CODE_GEN_ARG_PASTE9, \
        GEOMAG_CODE_GEN_ARG_PASTE8, \
        GEOMAG_CODE_GEN_ARG_PASTE7, \
        GEOMAG_CODE_GEN_ARG_PASTE6, \
        GEOMAG_CODE_GEN_ARG_PASTE5, \
        GEOMAG_CODE_GEN_ARG_PASTE4, \
        GEOMAG_CODE_GEN_ARG_PASTE3, \
        GEOMAG_CODE_GEN_ARG_PASTE2, \
        GEOMAG_CODE_GEN_ARG_PASTE1)(__VA_ARGS__))
#define GEOMAG_CODE_GEN_ARG_PASTE1
#define GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) operator_function(v1)
#define GEOMAG_CODE_GEN_ARG_PASTE3(operator_function, v1, v2) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v2)
#define GEOMAG_CODE_GEN_ARG_PASTE4(operator_function, v1, v2, v3) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE3(operator_function, v2, v3)
#define GEOMAG_CODE_GEN_ARG_PASTE5(operator_function, v1, v2, v3, v4) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE4(operator_function, v2, v3, v4)
#define GEOMAG_CODE_GEN_ARG_PASTE6(operator_function, v1, v2, v3, v4, v5) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE5(operator_function, v2, v3, v4, v5)
#define GEOMAG_CODE_GEN_ARG_PASTE7(operator_function, v1, v2, v3, v4, v5, v6) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE6(operator_function, v2, v3, v4, v5, v6)
#define GEOMAG_CODE_GEN_ARG_PASTE8(operator_function, v1, v2, v3, v4, v5, v6, v7) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE7(operator_function, v2, v3, v4, v5, v6, v7)
#define GEOMAG_CODE_GEN_ARG_PASTE9(operator_function, v1, v2, v3, v4, v5, v6, v7, v8) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE8(operator_function, v2, v3, v4, v5, v6, v7, v8)
#define GEOMAG_CODE_GEN_ARG_PASTE10(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE9(operator_function, v2, v3, v4, v5, v6, v7, v8, v9)
#define GEOMAG_CODE_GEN_ARG_PASTE11(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE10(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10)
#define GEOMAG_CODE_GEN_ARG_PASTE12(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE11(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11)
#define GEOMAG_CODE_GEN_ARG_PASTE13(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE12(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12)
#define GEOMAG_CODE_GEN_ARG_PASTE14(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE13(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13)
#define GEOMAG_CODE_GEN_ARG_PASTE15(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE14(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14)
#define GEOMAG_CODE_GEN_ARG_PASTE16(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE15(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15)
#define GEOMAG_CODE_GEN_ARG_PASTE17(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE16(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16)
#define GEOMAG_CODE_GEN_ARG_PASTE18(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE17(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17)
#define GEOMAG_CODE_GEN_ARG_PASTE19(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE18(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18)
#define GEOMAG_CODE_GEN_ARG_PASTE20(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE19(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19)
#define GEOMAG_CODE_GEN_ARG_PASTE21(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE20(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20)
#define GEOMAG_CODE_GEN_ARG_PASTE22(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE21(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21)
#define GEOMAG_CODE_GEN_ARG_PASTE23(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE22(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22)
#define GEOMAG_CODE_GEN_ARG_PASTE24(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE23(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23)
#define GEOMAG_CODE_GEN_ARG_PASTE25(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE24(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24)
#define GEOMAG_CODE_GEN_ARG_PASTE26(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE25(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25)
#define GEOMAG_CODE_GEN_ARG_PASTE27(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE26(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26)
#define GEOMAG_CODE_GEN_ARG_PASTE28(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE27(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27)
#define GEOMAG_CODE_GEN_ARG_PASTE29(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE28(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28)
#define GEOMAG_CODE_GEN_ARG_PASTE30(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE29(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29)
#define GEOMAG_CODE_GEN_ARG_PASTE31(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE30(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30)
#define GEOMAG_CODE_GEN_ARG_PASTE32(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE31(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31)
#define GEOMAG_CODE_GEN_ARG_PASTE33(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE32(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32)
#define GEOMAG_CODE_GEN_ARG_PASTE34(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE33(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33)
#define GEOMAG_CODE_GEN_ARG_PASTE35(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE34(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34)
#define GEOMAG_CODE_GEN_ARG_PASTE36(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE35(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35)
#define GEOMAG_CODE_GEN_ARG_PASTE37(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE36(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36)
#define GEOMAG_CODE_GEN_ARG_PASTE38(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE37(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37)
#define GEOMAG_CODE_GEN_ARG_PASTE39(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE38(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38)
#define GEOMAG_CODE_GEN_ARG_PASTE40(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE39(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39)
#define GEOMAG_CODE_GEN_ARG_PASTE41(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE40(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40)
#define GEOMAG_CODE_GEN_ARG_PASTE42(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE41(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41)
#define GEOMAG_CODE_GEN_ARG_PASTE43(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE42(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42)
#define GEOMAG_CODE_GEN_ARG_PASTE44(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE43(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43)
#define GEOMAG_CODE_GEN_ARG_PASTE45(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE44(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44)
#define GEOMAG_CODE_GEN_ARG_PASTE46(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE45(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45)
#define GEOMAG_CODE_GEN_ARG_PASTE47(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE46(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46)
#define GEOMAG_CODE_GEN_ARG_PASTE48(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE47(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47)
#define GEOMAG_CODE_GEN_ARG_PASTE49(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE48(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48)
#define GEOMAG_CODE_GEN_ARG_PASTE50(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE49(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49)
#define GEOMAG_CODE_GEN_ARG_PASTE51(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE50(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50)
#define GEOMAG_CODE_GEN_ARG_PASTE52(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE51(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51)
#define GEOMAG_CODE_GEN_ARG_PASTE53(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE52(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52)
#define GEOMAG_CODE_GEN_ARG_PASTE54(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE53(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53)
#define GEOMAG_CODE_GEN_ARG_PASTE55(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE54(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54)
#define GEOMAG_CODE_GEN_ARG_PASTE56(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE55(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55)
#define GEOMAG_CODE_GEN_ARG_PASTE57(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE56(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56)
#define GEOMAG_CODE_GEN_ARG_PASTE58(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE57(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57)
#define GEOMAG_CODE_GEN_ARG_PASTE59(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE58(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58)
#define GEOMAG_CODE_GEN_ARG_PASTE60(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE59(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59)
#define GEOMAG_CODE_GEN_ARG_PASTE61(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE60(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60)
#define GEOMAG_CODE_GEN_ARG_PASTE62(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE61(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61)
#define GEOMAG_CODE_GEN_ARG_PASTE63(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61, v62) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE62(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61, v62)
#define GEOMAG_CODE_GEN_ARG_PASTE64(operator_function, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61, v62, v63) GEOMAG_CODE_GEN_ARG_PASTE2(operator_function, v1) GEOMAG_CODE_GEN_ARG_PASTE63(operator_function, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28, v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43, v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58, v59, v60, v61, v62, v63)

// clang-format on
