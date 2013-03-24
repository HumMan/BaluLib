#pragma once

#include <string>

#define MAX_PATH 260

namespace UtfConverter
{
	template<class T, class A> std::wstring to_unicode(
        std::basic_string<char, T, A> const& in,
        std::locale const& loc = std::locale())
    {
        typedef std::codecvt<wchar_t, char, std::mbstate_t> facet;
        const facet& cvt = std::use_facet<facet>(loc);
    
        std::wstring out;
        out.reserve(in.length());
    
        facet::state_type state = facet::state_type();
        const char *ibuf = in.data(), *iend = in.data() + in.size();
        while(ibuf != iend)
        {
            wchar_t obuf[MAX_PATH], *oend;
            facet::result res = cvt.in (state, ibuf, iend, ibuf, obuf, obuf + MAX_PATH, oend = obuf);
            if( res == facet::error ) { state = facet::state_type(); ibuf += 1; }
            out.append(obuf, oend - obuf);
            if( res == facet::error ) out += L'?';
        }
        return out;
    }
    
    template<class T, class A> std::string to_multibyte(
        std::basic_string<wchar_t, T, A> const& in,
        std::locale const& loc = std::locale())
    {
        typedef std::codecvt<wchar_t,char,std::mbstate_t> facet;
        facet const& cvt = std::use_facet<facet>(loc);
    
        std::string out;
        out.reserve(in.length());
    
        facet::state_type state = facet::state_type();
        const wchar_t *ibuf = in.data(), *iend = in.data() + in.size();
        char obuf[MAX_PATH], *oend;
        while(ibuf != iend)
        {
            facet::result res = cvt.out(state, ibuf, iend, ibuf, obuf, obuf + MAX_PATH, oend = obuf);
            if( res == facet::error ) { state = facet::state_type(); ibuf += 1; }
            out.append(obuf, oend - obuf);
            if( res == facet::error ) out += L'?';
        }
        if( state == facet::partial )
        {
            cvt.unshift(state, obuf, obuf + MAX_PATH, oend = obuf);
            out.append (obuf, oend - obuf);
        }
        return out;
    }
}
#undef MAX_PATH 260