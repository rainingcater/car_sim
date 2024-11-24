/******************************************************************************
* Copyright 2017. All Rights Reserved.
*
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

/**
* @file
*/

#include "struct_endian.h"

int struct_get_endian(void)
{
	int i = 0x00000001;
	if (((char *)&i)[0]) {
		return STRUCT_ENDIAN_LITTLE;
	} else {
		return STRUCT_ENDIAN_BIG;
	}
}
