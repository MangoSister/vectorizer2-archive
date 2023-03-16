#pragma once

#ifdef _MSC_VER
#ifndef VECTORIZER_STATIC
#ifdef VECTORIZER_EXPORTS
#define VECTORIZER_API __declspec(dllexport)
#else
#define VECTORIZER_API __declspec(dllimport)
#endif
#else
#define VECTORIZER_API
#endif
#else
#define VECTORIZER_API
#endif