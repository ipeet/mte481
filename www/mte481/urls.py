from django.conf.urls.defaults import patterns, include, url

# Uncomment the next two lines to enable the admin:
# from django.contrib import admin
# admin.autodiscover()

urlpatterns = patterns('',
    url(r'^$', 'mte481.website.views.root'),
    url(r'^(?P<page>[^/]+)/?$', 'mte481.website.views.page'),
    url(r'^styles/(?P<style>[^./]+).css$', 'mte481.website.views.style'),
    # Examples:
    # url(r'^$', 'mte481.views.home', name='home'),
    # url(r'^mte481/', include('mte481.foo.urls')),

    # Uncomment the admin/doc line below to enable admin documentation:
    # url(r'^admin/doc/', include('django.contrib.admindocs.urls')),

    # Uncomment the next line to enable the admin:
    # url(r'^admin/', include(admin.site.urls)),
)
