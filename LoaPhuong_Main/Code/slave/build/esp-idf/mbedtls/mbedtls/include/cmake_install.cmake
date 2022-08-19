# Install script for directory: /mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/aesni.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/arc4.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/blowfish.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/bn_mul.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/certs.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/cipher_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/compat-1.3.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/config.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ecp_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/entropy_poll.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/havege.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/md2.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/md4.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/md_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/net.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/padlock.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pk_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pkcs11.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/rsa_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl_internal.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    "/mnt/e/esp/audio_ip_adf/fire_safe_esp32_sdk/components/mbedtls/mbedtls/include/mbedtls/xtea.h"
    )
endif()

