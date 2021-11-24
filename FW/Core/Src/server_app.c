/***********************************************************************************************************************
*                                                                                                                      *
* HackerspaceSG WS2812 Lighting Controller                                                                             *
*                                                                                                                      *
* Description: HTTP Server Application for LED control                                                                 *
* Author: Joyce Ng                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2021 Joyce Ng, HackerspaceSG                                                                           *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include <stdio.h>
#include "server_app.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "cmsis_os.h"

// Definition of Threads
osThreadId_t HTTPServerTaskHandle;

const osThreadAttr_t HTTPServerTask_attributes = {
    .name = "HTTPServerTask",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 4096
  };

// Start HTTP Server Thread
void http_server_init(void)
{
	osThreadNew(HTTPServerThread, NULL, &HTTPServerTask_attributes);
}

// Serve connections and parses any JSON data sent
void http_server_serve(struct netconn *conn)
{

}

// HTTP Server Thread
void HTTPServerThread(void *argument)
{
	struct netconn *conn, *newconn;
	err_t err, accept_err;

	conn = netconn_new(NETCONN_TCP);

	if(conn != NULL)
	{
		// Bind to Port 80
		err = netconn_bind(conn, NULL, 80);

		if(err == ERR_OK)
		{
		   // Listen out for any connections
		   netconn_listen(conn);

		   while(1)
		   {
		      // Accept any incoming connection
			  accept_err = netconn_accept(conn, &newconn);
			  if(accept_err == ERR_OK)
			  {
				 // Serve connection
				 http_server_serve(newconn);

				 // Delete connection
			     netconn_delete(newconn);
			  }

		   }
		}
	}
}
